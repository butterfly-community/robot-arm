//! Time pairing and decoding from retained H.264 GOPs. No device is reopened.
use super::*;
pub(super) struct Pair {
    pub(super) sources: [String; 2],
    pub(super) times: [i64; 2],
    pub(super) width: u32,
    pub(super) height: u32,
    pub(super) fps: u32,
    packets: [Vec<Arc<robot_arm_messages::CameraRawVideoFrame>>; 2],
}
impl Pair {
    pub(super) fn left(&self) -> Result<Vec<u8>> {
        camera_media::decode(self.packets[0].iter().map(|p| &p.color))
    }
    pub(super) fn right(&self) -> Result<Vec<u8>> {
        camera_media::decode(self.packets[1].iter().map(|p| &p.color))
    }
    pub(super) fn validate(&self, saved: &Saved) -> Result<()> {
        ensure!(self.sources == saved.sources, "相机绑定与标定不一致");
        ensure!(
            (self.width, self.height) == (saved.optics.width, saved.optics.height),
            "相机分辨率与标定不一致，请恢复标定时的流配置或重新标定"
        );
        Ok(())
    }
}
pub(super) fn pair(s: &Shared, rig: Rig) -> Result<Pair> {
    let roles = rig.roles();
    let get = |role| {
        s.videos
            .get(&role)
            .and_then(|v| v.borrow().clone())
            .ok_or_else(|| eyre!("请绑定并开启同组左右相机，等待两路图像信号"))
    };
    let l = get(roles[0])?;
    let r = get(roles[1])?;
    // Both watch values retain recent GOPs. Match capture receipt times, not encode completion.
    let packets = |v: &camera_media::VideoFrame| {
        v.previous_gop
            .iter()
            .chain(v.gop.iter())
            .cloned()
            .collect::<Vec<_>>()
    };
    let lp = packets(&l);
    let rp = packets(&r);
    let latest = lp
        .last()
        .ok_or_else(|| eyre!("等待左相机视频帧"))?
        .received_time_ns
        .min(
            rp.last()
                .ok_or_else(|| eyre!("等待右相机视频帧"))?
                .received_time_ns,
        );
    let li = lp
        .iter()
        .enumerate()
        .min_by_key(|(_, p)| p.received_time_ns.abs_diff(latest))
        .map(|(i, _)| i)
        .unwrap();
    let ri = rp
        .iter()
        .enumerate()
        .min_by_key(|(_, b)| lp[li].received_time_ns.abs_diff(b.received_time_ns))
        .map(|(i, _)| i)
        .ok_or_else(|| eyre!("等待右相机视频帧"))?;
    let a = &lp[li];
    let b = &rp[ri];
    ensure!(
        a.color.width == b.color.width && a.color.height == b.color.height,
        "左右相机需要选择相同图像尺寸"
    );
    let period = 1e9 / l.frames_per_second.min(r.frames_per_second);
    ensure!(
        (a.received_time_ns - b.received_time_ns).unsigned_abs() as f64 <= period,
        "左右图像时间差超过一个采集周期，请检查采集帧率及 USB 带宽"
    );
    let chain = |packets: &[Arc<robot_arm_messages::CameraRawVideoFrame>],
                 index: usize,
                 boundary: usize| {
        // Locate the containing GOP. The retained previous/current GOP each begins at IDR.
        let start = if index >= boundary { boundary } else { 0 };
        packets[start..=index].to_vec()
    };
    Ok(Pair {
        sources: [a.source_id.clone(), b.source_id.clone()],
        times: [a.received_time_ns, b.received_time_ns],
        width: a.color.width,
        height: a.color.height,
        fps: l.frames_per_second.min(r.frames_per_second).round() as u32,
        packets: [
            chain(&lp, li, l.previous_gop.len()),
            chain(&rp, ri, r.previous_gop.len()),
        ],
    })
}

pub(super) async fn fresh_pair(s: Arc<Shared>, rig: Rig) -> Result<Pair> {
    let mut a = s.videos[&rig.roles()[0]].clone();
    let mut b = s.videos[&rig.roles()[1]].clone();
    ensure!(
        a.borrow().is_some() && b.borrow().is_some(),
        "请绑定并开启同组左右相机"
    );
    let since = now_ns();
    loop {
        tokio::select! {v=a.changed()=>{v?;},v=b.changed()=>{v?;}}
        // Pair metadata first; discarded/old pairs never enter FFmpeg.
        let p = pair(&s, rig)?;
        if p.times.iter().all(|t| *t >= since) {
            return Ok(p);
        }
    }
}
