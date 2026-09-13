"use client";

import { useEffect, useRef, useState, type PointerEvent } from "react";

export type ImageBox = { id: string; label: string; bbox: number[] };

// Pixel coordinates always refer to the frozen source image, regardless of CSS size.
export function ImageBoxCanvas({
  src,
  width,
  height,
  boxes,
  disabled,
  label,
  onDraw,
  onSelect,
}: {
  src: string;
  width: number;
  height: number;
  boxes: ImageBox[];
  disabled: boolean;
  label: string;
  onDraw?: (bbox: [number, number, number, number]) => void;
  onSelect?: (id: string) => void;
}) {
  const [start, setStart] = useState<number[]>();
  const [cursor, setCursor] = useState<number[]>();
  const [loaded, setLoaded] = useState<string>();
  const svg = useRef<SVGSVGElement>(null);
  const [labelScale, setLabelScale] = useState(1);
  useEffect(() => {
    const element = svg.current;
    if (!element) return;
    const observer = new ResizeObserver(([entry]) => {
      if (entry.contentRect.width)
        setLabelScale(width / entry.contentRect.width);
    });
    observer.observe(element);
    return () => observer.disconnect();
  }, [width]);
  const point = (event: PointerEvent<SVGSVGElement>) => {
    const rect = event.currentTarget.getBoundingClientRect();
    return [
      Math.max(
        0,
        Math.min(width, ((event.clientX - rect.left) * width) / rect.width),
      ),
      Math.max(
        0,
        Math.min(height, ((event.clientY - rect.top) * height) / rect.height),
      ),
    ];
  };
  const box = (a: number[], b: number[]): [number, number, number, number] => [
    Math.min(a[0], b[0]),
    Math.min(a[1], b[1]),
    Math.max(a[0], b[0]),
    Math.max(a[1], b[1]),
  ];
  const shown = [
    ...boxes,
    ...(start && cursor
      ? [{ id: "draft", label: "", bbox: box(start, cursor) }]
      : []),
  ];
  return (
    <svg
      ref={svg}
      aria-label={label}
      role={onSelect ? "group" : "img"}
      viewBox={`0 0 ${width} ${height}`}
      style={{
        display: "block",
        width: "100%",
        touchAction: disabled ? "auto" : "none",
        cursor: disabled ? "default" : "crosshair",
      }}
      onPointerDown={(event) => {
        if (disabled || loaded !== src || event.button !== 0) return;
        event.currentTarget.setPointerCapture(event.pointerId);
        setStart(point(event));
        setCursor(point(event));
      }}
      onPointerMove={(event) => {
        if (start && !disabled) setCursor(point(event));
      }}
      onPointerCancel={() => {
        setStart(undefined);
        setCursor(undefined);
      }}
      onPointerUp={(event) => {
        if (start && !disabled && loaded === src) {
          const end = point(event);
          if (end[0] !== start[0] && end[1] !== start[1])
            onDraw?.(box(start, end));
        }
        setStart(undefined);
        setCursor(undefined);
      }}
    >
      <image
        href={src}
        width={width}
        height={height}
        onLoad={() => setLoaded(src)}
      />
      {shown.map((item) => (
        <g key={item.id}>
          <title>{item.label}</title>
          <rect
            x={item.bbox[0]}
            y={item.bbox[1]}
            width={item.bbox[2] - item.bbox[0]}
            height={item.bbox[3] - item.bbox[1]}
            fill="none"
            stroke="#45d6c1"
            strokeWidth="2"
            vectorEffect="non-scaling-stroke"
          />
          <foreignObject
            x={item.bbox[0] + 2}
            y={Math.max(
              0,
              Math.min(item.bbox[1] + 2, height - 24 * labelScale),
            )}
            width={Math.max(
              0,
              Math.min(
                Math.max(100 * labelScale, item.bbox[2] - item.bbox[0] - 4),
                width - item.bbox[0] - 2,
              ),
            )}
            height={24 * labelScale}
          >
            <div
              title={item.label}
              style={{
                color: "#45d6c1",
                fontSize: 12 * labelScale,
                whiteSpace: "nowrap",
                overflow: "hidden",
                textOverflow: "ellipsis",
                background: "#101419d9",
                lineHeight: `${22 * labelScale}px`,
                paddingInline: 2,
              }}
            >
              {onSelect ? (
                <button
                  type="button"
                  className="annotation-title"
                  aria-label={`查看分割详情 ${item.label}`}
                  onPointerDown={(event) => event.stopPropagation()}
                  onClick={() => onSelect(item.id)}
                >
                  {item.label}
                </button>
              ) : (
                item.label
              )}
            </div>
          </foreignObject>
        </g>
      ))}
    </svg>
  );
}
