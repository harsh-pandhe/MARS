# Post 6 — When PyTorch's Optimizer Crashes Before Training Even Starts

**Attach:** none needed — code-quote style post, maybe a screenshot of the stack trace

---

Segmentation fault. Inside `torch.optim.Adam.__init__`. Before a single training step ran.

That's a special kind of frustrating bug — it's not your model, it's not your data, it's not even your training loop. It's the optimizer's constructor crashing at the C++ level.

Root cause: PyTorch's TorchDynamo compiler was trying to trace/compile the Adam initializer inside a Ray worker thread, and something about that threading context caused a hard crash rather than a clean Python exception.

The fix was one environment variable: `TORCHDYNAMO_DISABLE=1`.

One line. Hours of debugging. This is most of what real ML engineering actually looks like — not model architecture, but chasing down which layer of your six-deep dependency stack is fighting with which other layer.

#PyTorch #MachineLearning #Debugging #DeepLearning
