import { Slot } from "@radix-ui/react-slot";
import { cva, type VariantProps } from "class-variance-authority";
import { clsx, type ClassValue } from "clsx";
import type {
  ButtonHTMLAttributes,
  HTMLAttributes,
  InputHTMLAttributes,
  ReactNode,
} from "react";
import { twMerge } from "tailwind-merge";

export function cn(...values: ClassValue[]) {
  return twMerge(clsx(values));
}

const buttonVariants = cva("button", {
  variants: {
    variant: {
      default: "button-default",
      outline: "button-outline",
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
  children,
  className,
  ...props
}: HTMLAttributes<HTMLElement> & { title: string }) {
  return (
    <section {...props} className={cn("card", className)}>
      <h2>{title}</h2>
      {children}
    </section>
  );
}
export function Field({
  label,
  children,
}: {
  label: string;
  children: ReactNode;
}) {
  return (
    <fieldset className="field">
      <legend>{label}</legend>
      {children}
    </fieldset>
  );
}
export function JsonView({ value }: { value: unknown }) {
  return (
    <pre className="json" tabIndex={0}>
      {JSON.stringify(value ?? null, null, 2)}
    </pre>
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
  return items.map((item) => (
    <label className="field" key={item.key}>
      <span>
        {item.label} · {(values[item.key] ?? 0).toFixed(3)} {item.unit}
      </span>
      <input
        type="range"
        min={item.minimum}
        max={item.maximum}
        step="any"
        value={values[item.key] ?? 0}
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
    </label>
  ));
}
export function Shell({
  title,
  description,
  children,
}: {
  title: string;
  description: string;
  children: ReactNode;
}) {
  return (
    <main>
      <nav aria-label="服务导航">
        <a href="/tracking/">采集</a>
        <a href="/spatial/">空间</a>
        <a href="/motion/">运动</a>
        <a href="/arm-execution/">执行</a>
      </nav>
      <header>
        <p className="eyebrow">DORA ROBOT SERVICES</p>
        <h1>{title}</h1>
        <p>{description}</p>
      </header>
      {children}
    </main>
  );
}
