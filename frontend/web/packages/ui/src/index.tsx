"use client";

import { virtualFeedbackTarget, type ActionFeedback } from "@robot/contracts";
import { useGateway } from "@robot/gateway-client";
import * as Collapsible from "@radix-ui/react-collapsible";
import { Slot } from "@radix-ui/react-slot";
import { cva, type VariantProps } from "class-variance-authority";
import { clsx, type ClassValue } from "clsx";
import type {
  ButtonHTMLAttributes,
  CSSProperties,
  HTMLAttributes,
  InputHTMLAttributes,
  ReactNode,
} from "react";
import { useRef, useState } from "react";

export function cn(...values: ClassValue[]) {
  return clsx(values);
}

const buttonVariants = cva("button", {
  variants: {
    variant: {
      default: "button-default",
      outline: "button-outline",
      danger: "button-danger",
    },
  },
  defaultVariants: { variant: "default" },
});

export function Button({
  asChild = false,
  variant,
  className,
  ...props
}: ButtonHTMLAttributes<HTMLButtonElement> &
  VariantProps<typeof buttonVariants> & { asChild?: boolean }) {
  const Component = asChild ? Slot : "button";
  return (
    <Component
      {...props}
      className={cn(buttonVariants({ variant }), className)}
    />
  );
}

export function Input({
  className,
  ...props
}: InputHTMLAttributes<HTMLInputElement>) {
  return <input {...props} className={cn("input", className)} />;
}

export function Card({
  title,
  eyebrow,
  action,
  defaultOpen = true,
  children,
  className,
  ...props
}: HTMLAttributes<HTMLElement> & {
  title: string;
  eyebrow?: string;
  action?: ReactNode;
  defaultOpen?: boolean;
}) {
  return (
    <Collapsible.Root defaultOpen={defaultOpen} asChild>
      <section {...props} className={cn("card", className)}>
        <div className="card-heading">
          <Collapsible.Trigger className="card-toggle">
            <span className="card-title">
              <LocalizedLabel
                text={title}
                english={eyebrow}
                className="card-title-text"
              />
            </span>
            <span className="card-chevron" aria-hidden="true" />
          </Collapsible.Trigger>
          {action && <div className="card-action">{action}</div>}
        </div>
        <Collapsible.Content className="card-content">
          <div className="card-content-inner">{children}</div>
        </Collapsible.Content>
      </section>
    </Collapsible.Root>
  );
}

export function Field({
  label,
  englishLabel,
  hint,
  children,
}: {
  label: string;
  englishLabel?: string;
  hint?: string;
  children: ReactNode;
}) {
  return (
    <fieldset className="field">
      <legend>
        <LocalizedLabel text={label} english={englishLabel} />
      </legend>
      {children}
      {hint && <small>{hint}</small>}
    </fieldset>
  );
}

export function JsonView({
  value,
  title = "原始数据",
  englishTitle,
}: {
  value: unknown;
  title?: string;
  englishTitle?: string;
}) {
  return (
    <details className="diagnostics">
      <summary>
        <LocalizedLabel text={title} english={englishTitle} />
      </summary>
      <pre className="json" tabIndex={0}>
        {JSON.stringify(value ?? null, null, 2)}
      </pre>
    </details>
  );
}

export function StatusBadge({
  children,
  tone = "neutral",
}: {
  children: ReactNode;
  tone?: "neutral" | "good" | "warning" | "bad" | "cyan";
}) {
  return (
    <span className={`badge badge-${tone}`}>
      <LocalizedLabel text={children} />
    </span>
  );
}

export function Metric({
  label,
  englishLabel,
  value,
  unit,
  tone,
}: {
  label: string;
  englishLabel?: string;
  value: ReactNode;
  unit?: string;
  tone?: "cyan" | "red" | "green" | "blue" | "amber";
}) {
  return (
    <div className={cn("metric", tone && `metric-${tone}`)}>
      <span>
        <LocalizedLabel text={label} english={englishLabel} />
      </span>
      <strong>
        <LocalizedLabel text={value} />
      </strong>
      {unit && <small>{unit}</small>}
    </div>
  );
}

export function KeyValue({
  label,
  englishLabel,
  value,
}: {
  label: string;
  englishLabel?: string;
  value: ReactNode;
}) {
  return (
    <div className="key-value">
      <span>
        <LocalizedLabel text={label} english={englishLabel} />
      </span>
      <strong>
        <LocalizedLabel text={value} />
      </strong>
    </div>
  );
}

export type RangeItem = {
  key: string;
  label: string;
  unit: string;
  minimum: number;
  maximum: number;
};

export function RangeControls({
  items,
  values,
  onBegin,
  onChange,
  onCommit,
}: {
  items: RangeItem[];
  values: Record<string, number>;
  onBegin?: (key: string) => void;
  onChange: (key: string, value: number) => void;
  onCommit: (key: string, value: number) => void;
}) {
  return (
    <div className="range-stack">
      {items.map((item) => {
        const value = values[item.key] ?? 0;
        return (
          <label className="range-control" key={item.key}>
            <span className="range-name">{item.label}</span>
            <input
              type="range"
              min={item.minimum}
              max={item.maximum}
              step="any"
              value={value}
              onPointerDown={() => onBegin?.(item.key)}
              onChange={(event) =>
                onChange(item.key, event.currentTarget.valueAsNumber)
              }
              onPointerUp={(event) =>
                onCommit(item.key, event.currentTarget.valueAsNumber)
              }
              onKeyUp={(event) => {
                if (event.key.startsWith("Arrow"))
                  onCommit(item.key, event.currentTarget.valueAsNumber);
              }}
            />
            <output>
              {item.unit === "rad"
                ? `${((value * 180) / Math.PI).toFixed(1)}°`
                : `${value.toFixed(3)} ${item.unit}`}
            </output>
          </label>
        );
      })}
    </div>
  );
}

const navigation = [
  ["/tracking/", "采集", "Input", "01"],
  ["/spatial/", "空间", "Spatial", "02"],
  ["/motion/", "运动", "Motion", "03"],
  ["/arm-execution/", "执行", "Execution", "04"],
] as const;

function VirtualFeedbackOverlay() {
  const { snapshot } = useGateway("tracking");
  const discovery = snapshot?.values.discovery_state as
    Record<string, unknown> | undefined;
  const feedback = discovery?.virtual_feedback as ActionFeedback | undefined;
  const selected = (
    (discovery?.feedback_bindings ?? []) as Array<Record<string, unknown>>
  ).some(
    (binding) =>
      binding.source_id === virtualFeedbackTarget.sourceId &&
      binding.capability_path === virtualFeedbackTarget.capabilityPath,
  );
  const [position, setPosition] = useState<{ left: number; top: number }>();
  const dragOffset = useRef<{ x: number; y: number } | undefined>(undefined);

  if (!selected) return null;

  const value = Math.round(feedback?.strength_percent ?? 0);
  const style: CSSProperties & { "--feedback-value": string } = {
    "--feedback-value": `${value * 3.6}deg`,
    ...(position ?? { right: 18, bottom: 18 }),
  };

  return (
    <div
      className="virtual-feedback"
      role="meter"
      aria-label="网页虚拟力度反馈"
      aria-valuemin={0}
      aria-valuemax={100}
      aria-valuenow={value}
      style={style}
      onPointerDown={(event) => {
        const bounds = event.currentTarget.getBoundingClientRect();
        dragOffset.current = {
          x: event.clientX - bounds.left,
          y: event.clientY - bounds.top,
        };
        event.currentTarget.setPointerCapture(event.pointerId);
      }}
      onPointerMove={(event) => {
        if (!dragOffset.current) return;
        setPosition({
          left: event.clientX - dragOffset.current.x,
          top: event.clientY - dragOffset.current.y,
        });
      }}
      onPointerUp={() => {
        dragOffset.current = undefined;
      }}
      onPointerCancel={() => {
        dragOffset.current = undefined;
      }}
    >
      <span>{value}</span>
    </div>
  );
}

export function Shell({
  title,
  description,
  section,
  children,
}: {
  title: string;
  description: string;
  section?: string;
  children: ReactNode;
}) {
  const [sectionIndex, sectionEnglish] = section?.split(" / ", 2) ?? [
    "服务",
    "Control service",
  ];

  return (
    <main className="console-shell">
      <header className="topbar">
        <div className="brand">
          <span className="brand-mark" aria-hidden="true">
            <svg viewBox="0 0 32 32">
              <path d="M5 27h17M8 27v-5h8v5M12 22l4-9 7 4M16 13l5-7M21 6h5M26 3v6M26 6h3" />
              <circle cx="12" cy="22" r="2" />
              <circle cx="16" cy="13" r="2" />
              <circle cx="23" cy="17" r="2" />
            </svg>
          </span>
          <div>
            <strong>
              <LocalizedLabel text="机械臂控制台" english="Robot Arm Console" />
            </strong>
            <small>
              <LocalizedLabel
                text="Dora 服务网络"
                english="Dora Control Fabric"
              />
            </small>
          </div>
        </div>
        <nav aria-label="服务导航">
          {navigation.map(([href, label, english, index]) => (
            <a
              key={href}
              href={href}
              aria-current={section?.startsWith(index) ? "page" : undefined}
            >
              <span>{index}</span>
              <LocalizedLabel text={label} english={english} />
            </a>
          ))}
        </nav>
        <div className="topbar-state">
          <i />
          <LocalizedLabel text="服务在线" english="Services online" />
        </div>
      </header>
      <div className="page-heading">
        <div>
          <p className="eyebrow">
            <LocalizedLabel text={sectionIndex} english={sectionEnglish} />
          </p>
          <h1>{title}</h1>
        </div>
        <p>{description}</p>
      </div>
      {children}
      <VirtualFeedbackOverlay />
    </main>
  );
}

const interfaceTerms: Record<string, readonly [string, string]> = {
  "Position X": ["位置 X", "Position X"],
  "Position Y": ["位置 Y", "Position Y"],
  "Position Z": ["位置 Z", "Position Z"],
  "Pose tracking": ["位姿跟踪", "Pose tracking"],
  Control: ["控制状态", "Control"],
  "Observed rate": ["采集频率", "Observed rate"],
  "Control mode": ["控制模式", "Control mode"],
  Feedback: ["反馈来源", "Feedback"],
  "MoveIt state": ["MoveIt 状态", "MoveIt state"],
  "TCP X": ["末端 X", "TCP X"],
  "TCP Y": ["末端 Y", "TCP Y"],
  "TCP Z": ["末端 Z", "TCP Z"],
  Transport: ["连接方式", "Transport"],
  Endpoint: ["连接端点", "Endpoint"],
  "Command seq": ["命令序号", "Command sequence"],
  "Joint count": ["关节数量", "Joint count"],
  "Parameter values": ["已读取参数", "Parameter values"],
  TRACKED: ["已跟踪", "Tracked"],
  WAIT: ["等待", "Wait"],
  ACTIVE: ["已启用", "Active"],
  IDLE: ["空闲", "Idle"],
  RELATIVE: ["相对控制", "Relative control"],
  relative: ["相对控制", "Relative control"],
  MANUAL: ["手动控制", "Manual control"],
  manual: ["手动控制", "Manual control"],
  PLANNING: ["规划中", "Planning"],
  EXECUTING: ["执行中", "Executing"],
  SUCCEEDED: ["已完成", "Succeeded"],
  CANCELLED: ["已取消", "Cancelled"],
  FAILED: ["失败", "Failed"],
  SOFTWARE: ["软件反馈", "Software feedback"],
  HARDWARE: ["真机反馈", "Hardware feedback"],
  CONNECTED: ["已连接", "Connected"],
  SIMULATION: ["模拟", "Simulation"],
  servo_status_code: ["Servo 状态码", "Servo status code"],
  servo_status_message: ["Servo 状态说明", "Servo status message"],
  "Moving closer to a singularity, decelerating": [
    "接近奇异位形，正在减速",
    "Moving closer to a singularity, decelerating",
  ],
  "Moving away from a singularity, decelerating": [
    "正在远离奇异位形并减速",
    "Moving away from a singularity, decelerating",
  ],
  左右: ["左右", "Left / right"],
  上下: ["上下", "Up / down"],
  前后: ["前后", "Forward / backward"],
  "前部抬起 / 往下": ["前部抬起 / 往下", "Front pitch"],
  "左旋 / 右旋": ["左旋 / 右旋", "Horizontal arc"],
  驱动: ["驱动", "Driver"],
  设备: ["设备", "Device"],
  当前: ["当前", "Current"],
  请求: ["请求", "Request"],
  结果: ["结果", "Result"],
  轨迹点: ["轨迹点", "Trajectory points"],
  位置: ["位置", "Position"],
  "姿态 XYZW": ["姿态 XYZW", "Orientation XYZW"],
  已连接: ["已连接", "Connected"],
  反馈: ["反馈", "Feedback"],
  当前反馈: ["当前反馈", "Current feedback"],
  最后命令: ["最后命令", "Last command"],
  平移倍率: ["平移倍率", "Translation scale"],
  "输入坐标 → 前 / 左 / 上坐标映射": [
    "输入坐标 → 前 / 左 / 上坐标映射",
    "Input coordinate mapping",
  ],
  采集分量: ["采集分量", "Captured components"],
  无绝对位置时的平移速度: [
    "无绝对位置时的平移速度",
    "Translation speed without absolute position",
  ],
  无绝对姿态时的圆弧角速度: [
    "无绝对姿态时的圆弧角速度",
    "Arc speed without absolute orientation",
  ],
  "MotionState 原始数据": ["运动状态原始数据", "Raw MotionState"],
  "Transport 原始状态": ["连接原始状态", "Raw transport state"],
  "唯一 ArmState": ["机械臂状态原始数据", "Raw ArmState"],
  "最后 ArmCommand": ["最后命令原始数据", "Raw ArmCommand"],
  "原始 Action": ["原始功能输入", "Raw actions"],
};

export function LocalizedLabel({
  text,
  english,
  className,
}: {
  text: ReactNode;
  english?: string;
  className?: string;
}) {
  const source = typeof text === "string" ? text : undefined;
  const localized = source ? interfaceTerms[source] : undefined;
  const visible = localized?.[0] ?? text;
  const translation = english ?? localized?.[1];

  return (
    <span className={cn("localized-label", className)}>
      <span>{visible}</span>
      {translation && (
        <span
          className="translation-dot"
          data-english={translation}
          aria-label={`英文：${translation}`}
          title={translation}
        />
      )}
    </span>
  );
}
