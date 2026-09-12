# Post 5 — The Bug Was My Wi-Fi

**Attach:** none needed, or a simple terminal-log screenshot of the crash trace

---

Multi-robot RL training crashed at iteration 34 of 45. No code changes, no memory leak, no obvious cause. It just... died.

Three hours of debugging later: my Wi-Fi adapter briefly dropped connection. That's it. That's the whole bug.

Here's the chain of failure: Ray (the distributed training framework) binds its internal coordination service to whatever IP address your machine's network interface reports — even for a single-machine job that never needed a network at all. When my Wi-Fi hiccuped for a few seconds, that IP briefly disappeared, Ray's internal processes lost contact with each other, and 34 iterations of training got thrown away.

The fix: force Ray to bind to localhost (127.0.0.1) explicitly instead of trusting the network stack. Training then ran to completion without another hiccup.

Infrastructure bugs like this are invisible in every paper's methods section, but they eat real engineering time. Sharing this so the next person doesn't burn three hours on it.

#MachineLearning #DistributedSystems #DebuggingStories #MLOps
