# Detection Runtime

Place the board-side inference implementation here.

Recommended responsibilities:

1. subscribe to or capture the stereo image;
2. preprocess once;
3. execute the quantized RDK BPU model;
4. decode outputs and perform NMS;
5. estimate distance from stereo depth;
6. publish one `PerceptionEvent` per valid target;
7. expose FPS, latency and dropped-frame diagnostics.

Training notebooks and model conversion scripts should remain separate from runtime code.
