// Shared user-path helpers: open the menu and click the visible DOM option.
export async function openOptions(trigger) {
  await trigger.click();
  const popup = trigger.page().getByRole("listbox");
  await popup.waitFor({ state: "visible" });
  return popup;
}

export async function chooseOption(trigger, choice) {
  const popup = await openOptions(trigger);
  const option =
    typeof choice === "object" && choice.label !== undefined
      ? popup.getByRole("option", { name: choice.label, exact: true })
      : popup.locator(
          `[role="option"][data-value=${JSON.stringify(String(typeof choice === "object" ? choice.value : choice))}]`,
        );
  await option.click();
}

export async function readOptions(trigger) {
  const popup = await openOptions(trigger);
  const options = await popup.getByRole("option").evaluateAll((items) =>
    items.map((el) => ({
      value: el.getAttribute("data-value"),
      text: el.textContent?.replace(/✓$/, "").trim(),
      disabled: el.getAttribute("aria-disabled") === "true",
    })),
  );
  await trigger.press("Escape");
  return options;
}
