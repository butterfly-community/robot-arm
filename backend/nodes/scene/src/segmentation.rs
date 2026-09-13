//! One frozen RGB-D capture, multiple model layers and an explicit rectangular manual layer.
//! These all produce the same DetectedInstance2D masks consumed by scene-core.
use super::*;

#[derive(Clone)]
pub(super) struct SegmentationLayer {
    pub source: String,
    pub instances: Vec<DetectedInstance2D>,
    pub placement_labels: Vec<String>,
}

pub(super) fn empty_segmented(
    frame: CameraFrameBundle,
    sequence: u64,
    model: String,
    models: Vec<PerceptionModelInfo>,
) -> SegmentedFrame {
    SegmentedFrame {
        sequence,
        model,
        models,
        frame,
        tool: None,
        instances: vec![],
        placement_labels: vec![],
        assets: BTreeMap::new(),
        layers: vec![],
        manual_regions: vec![],
    }
}

fn manual_instance(region: &ManualRegion, width: u32, height: u32) -> Result<DetectedInstance2D> {
    let [left, top, right, bottom] = region.bounding_box_xyxy;
    eyre::ensure!(
        !region.id.trim().is_empty() && !region.label.trim().is_empty(),
        "标注编号和名称不能为空"
    );
    eyre::ensure!(
        region.bounding_box_xyxy.iter().all(|v| v.is_finite())
            && 0. <= left
            && left < right
            && right <= width as f64
            && 0. <= top
            && top < bottom
            && bottom <= height as f64,
        "标注框必须在当前图像范围内，并具有正宽高"
    );
    // Pixel-centre inclusion; rectangle is user-specified, not a model contour.
    let mask = image::GrayImage::from_fn(width, height, |x, y| {
        image::Luma([
            if x as f64 + 0.5 >= left
                && (x as f64 + 0.5) < right
                && y as f64 + 0.5 >= top
                && (y as f64 + 0.5) < bottom
            {
                255
            } else {
                0
            },
        ])
    });
    let mut bytes = Cursor::new(Vec::new());
    DynamicImage::ImageLuma8(mask).write_to(&mut bytes, ImageFormat::Png)?;
    Ok(DetectedInstance2D {
        instance_id: format!("manual:{}", region.id),
        label: region.label.trim().into(),
        confidence: 1.,
        bounding_box_xyxy: region.bounding_box_xyxy,
        mask_width: width,
        mask_height: height,
        mask_png: bytes.into_inner(),
    })
}

pub(super) fn rebuild_segmented(mut input: SegmentedFrame) -> Result<SegmentedFrame> {
    input.instances = input
        .layers
        .iter()
        .flat_map(|layer| layer.instances.clone())
        .collect();
    input.placement_labels = input
        .layers
        .iter()
        .flat_map(|layer| layer.placement_labels.clone())
        .collect();
    let mut ids = std::collections::BTreeSet::new();
    for region in &input.manual_regions {
        eyre::ensure!(ids.insert(&region.id), "标注编号重复");
        let instance = manual_instance(region, input.frame.color.width, input.frame.color.height)?;
        input.placement_labels.push(instance.label.clone());
        input.instances.push(instance);
    }
    input.placement_labels.sort();
    input.placement_labels.dedup();
    let color = &input.frame.color;
    let overlay = segmentation_debug_image(color, &input.instances)?;
    let rgb = color_png(color)?;
    input.assets = BTreeMap::from([
        ("color.png".into(), ("image/png".into(), rgb.clone())),
        ("segmentation-color.png".into(), ("image/png".into(), rgb)),
        (
            "overlay.png".into(),
            ("image/png".into(), color_png(&overlay)?),
        ),
    ]);
    for (index, instance) in input.instances.iter().enumerate() {
        input.assets.insert(
            format!("mask-{index}.png"),
            ("image/png".into(), instance.mask_png.clone()),
        );
    }
    Ok(input)
}

pub(super) async fn edit_segmented(
    http: reqwest::Client,
    config: SceneConfig,
    mut input: SegmentedFrame,
    edit: SegmentationEdit,
    sequence: u64,
) -> Result<SegmentedFrame> {
    match edit {
        SegmentationEdit::Model => {
            // Never fetch another camera frame: RGB, depth, calibration and feedback stay paired.
            let result = segment_frame(
                http,
                config,
                input.frame.clone(),
                input.tool.clone(),
                sequence,
            )
            .await?;
            input.models = result.models;
            for layer in result.layers {
                input.layers.retain(|old| old.source != layer.source);
                input.layers.push(layer);
            }
        }
        SegmentationEdit::Manual { regions } => input.manual_regions = regions,
        SegmentationEdit::Remove { instance_ids } => {
            for id in &instance_ids {
                eyre::ensure!(
                    input.instances.iter().any(|item| &item.instance_id == id),
                    "待移除实例已不存在"
                );
            }
            for layer in &mut input.layers {
                layer
                    .instances
                    .retain(|item| !instance_ids.contains(&item.instance_id));
            }
            input
                .manual_regions
                .retain(|r| !instance_ids.contains(&format!("manual:{}", r.id)));
        }
        SegmentationEdit::Capture => bail!("载入新帧不能复用旧标注"),
    }
    input.sequence = sequence;
    tokio::task::spawn_blocking(move || rebuild_segmented(input))
        .await
        .context("合并分割结果失败")?
}
