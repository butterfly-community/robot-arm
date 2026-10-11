// Public browser API used here; codec parsing/muxing stays in the npm library.
declare module "jmuxer" {
  export default class JMuxer {
    constructor(options: {
      node: HTMLVideoElement;
      mode: "video";
      videoCodec: "H264";
      flushingTime: number;
      maxDelay: number;
      clearBuffer: boolean;
      live: boolean;
      onReady: () => void;
      onUnsupportedCodec: () => void;
      onError: (error: unknown) => void;
    });
    feed(data: {
      video: Uint8Array;
      duration: number;
      isLastVideoFrameComplete: boolean;
    }): void;
    destroy(): void;
  }
}
