import React, { useState } from "react";
import { createRoot } from "react-dom/client";
import { CameraVideo } from "../../web/apps/perception/src/app/camera-video";
import type { CameraRole } from "@robot/contracts";

function Preview() {
  const [visible, show] = useState(true);
  const [role, setRole] = useState<CameraRole>("external");
  return (
    <>
      <select
        aria-label="相机角色"
        value={role}
        onChange={(e) => setRole(e.target.value as CameraRole)}
      >
        <option value="depth">深度</option>
        <option value="external">外部</option>
        <option value="wrist">腕部</option>
      </select>
      <button onClick={() => show(!visible)}>
        {visible ? "收起" : "展开"}
      </button>
      {visible && <CameraVideo streaming role={role} />}
    </>
  );
}
createRoot(document.getElementById("root")!).render(<Preview />);
