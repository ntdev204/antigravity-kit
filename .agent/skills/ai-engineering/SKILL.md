---
name: ai-engineering
description: AI Engineering principles and decision-making for ML, DL, RL, and DRL. Framework selection, model architecture, training patterns, evaluation strategies, and deployment. Suitable from beginner to expert level. Use when working with machine learning, deep learning, reinforcement learning, model training, AI deployment, or MLOps tasks.
---

# AI Engineering

> Principles for building AI systems from fundamentals to production.

---

## 1. Framework Selection

### When to Use What

| Framework | Best For | Strengths |
|-----------|----------|-----------|
| **PyTorch** | Research, prototyping, production | Dynamic graphs, pythonic, strong ecosystem |
| **TensorFlow** | Production, mobile, large-scale | Mature deployment, TF Serving, TFLite |
| **JAX** | High-performance research | JIT compilation, auto-differentiation, functional |
| **scikit-learn** | Traditional ML | Simple API, comprehensive algorithms |
| **Keras** | Beginners, rapid prototyping | High-level, easy to learn |

### Decision Framework

| Need | Choose |
|------|--------|
| Research flexibility | PyTorch, JAX |
| Production deployment | TensorFlow, PyTorch (TorchServe) |
| Mobile/edge | TensorFlow Lite, ONNX |
| Traditional ML | scikit-learn |
| High-performance computing | JAX |

---

## 2. Machine Learning Fundamentals

### Learning Paradigms

| Type | Use Cases | Key Algorithms |
|------|-----------|----------------|
| **Supervised** | Classification, regression | Linear/Logistic Regression, SVM, Decision Trees, Random Forest, Gradient Boosting |
| **Unsupervised** | Clustering, dimensionality reduction | K-Means, DBSCAN, PCA, t-SNE, UMAP |
| **Semi-supervised** | Limited labeled data | Self-training, co-training, pseudo-labeling |

### Algorithm Selection

| Problem | Algorithm | When to Use |
|---------|-----------|-------------|
| **Binary Classification** | Logistic Regression | Linear relationships, interpretability needed |
| | SVM | Small datasets, non-linear boundaries |
| | Random Forest | Feature importance, robust to outliers |
| | XGBoost/LightGBM | Tabular data, high performance |
| **Regression** | Linear Regression | Linear relationships |
| | Random Forest Regressor | Non-linear, feature importance |
| | Neural Networks | Complex patterns, large data |
| **Clustering** | K-Means | Known cluster count, spherical clusters |
| | DBSCAN | Unknown cluster count, arbitrary shapes |

---

## 3. Deep Learning Architectures

### Core Architectures

| Architecture | Use Cases | Key Characteristics |
|--------------|-----------|---------------------|
| **CNN** | Computer vision, image processing | Spatial hierarchies, translation invariance |
| **RNN/LSTM/GRU** | Sequential data, time series | Temporal dependencies, variable length |
| **Transformer** | NLP, sequence modeling, vision | Self-attention, parallel processing |
| **GAN** | Generation, data augmentation | Adversarial training, realistic synthesis |
| **Autoencoder** | Compression, denoising, anomaly detection | Unsupervised feature learning |
| **VAE** | Generation, latent representation | Probabilistic encoding |

### Architecture Selection

| Task | Recommended Architecture |
|------|-------------------------|
| Image classification | CNN (ResNet, EfficientNet, ViT) |
| Object detection | CNN + Detection head (YOLO, Faster R-CNN) |
| Semantic segmentation | U-Net, DeepLab, Mask R-CNN |
| Text classification | Transformer (BERT, RoBERTa) |
| Sequence-to-sequence | Transformer (T5, BART) |
| Image generation | GAN (StyleGAN, Stable Diffusion), VAE |
| Time series forecasting | LSTM, Temporal CNN, Transformer |

---

## 4. Reinforcement Learning

### Core Concepts

| Concept | Description |
|---------|-------------|
| **Agent** | Decision-maker |
| **Environment** | What agent interacts with |
| **State (s)** | Current situation |
| **Action (a)** | Agent's choice |
| **Reward (r)** | Feedback signal |
| **Policy (π)** | Strategy mapping states to actions |
| **Value Function (V)** | Expected cumulative reward |

### Classical RL Algorithms

| Algorithm | Type | Use Cases |
|-----------|------|-----------|
| **Q-Learning** | Value-based | Discrete actions, tabular |
| **SARSA** | Value-based | On-policy, safer exploration |
| **Policy Gradient** | Policy-based | Continuous actions |
| **Actor-Critic** | Hybrid | Balance of both approaches |

---

## 5. Deep Reinforcement Learning

### Key Algorithms

| Algorithm | Type | Strengths | Use Cases |
|-----------|------|-----------|-----------|
| **DQN** | Value-based | Sample efficient | Discrete actions, games |
| **PPO** | Policy gradient | Stable, robust | Continuous control, robotics |
| **A3C/A2C** | Actor-critic | Parallel training | General purpose |
| **SAC** | Actor-critic | Sample efficient | Continuous control |
| **TD3** | Actor-critic | Handles overestimation | Continuous control |
| **DDPG** | Actor-critic | Deterministic policy | Continuous actions |

### Algorithm Selection

| Scenario | Recommended Algorithm |
|----------|----------------------|
| Discrete action space | DQN, Rainbow DQN |
| Continuous action space | PPO, SAC, TD3 |
| Need sample efficiency | SAC, TD3 |
| Distributed training | A3C, IMPALA |
| Stable training | PPO |

---

## 6. Training Best Practices

### Hyperparameter Tuning

| Strategy | When to Use |
|----------|-------------|
| **Grid Search** | Small search space, few parameters |
| **Random Search** | Better than grid for high dimensions |
| **Bayesian Optimization** | Expensive evaluation, continuous parameters |
| **Hyperband** | Early stopping, resource allocation |

### Regularization Techniques

| Technique | Purpose | Use Cases |
|-----------|---------|-----------|
| **Dropout** | Prevent overfitting | Deep networks, large models |
| **L1/L2** | Feature selection / weight decay | Linear models, general |
| **Batch Normalization** | Training stability | Deep networks |
| **Data Augmentation** | Increase diversity | Vision, limited data |
| **Early Stopping** | Prevent overfitting | All models |

### Optimization

| Optimizer | Characteristics | When to Use |
|-----------|----------------|-------------|
| **SGD** | Simple, momentum variants | When tuned well, often best |
| **Adam** | Adaptive learning rates | Default choice, fast convergence |
| **AdamW** | Weight decay fix | Transformers, modern architectures |
| **RMSprop** | Adaptive | RNNs |

---

## 7. Model Evaluation

### Metrics by Task

| Task | Primary Metrics | Secondary Metrics |
|------|----------------|-------------------|
| **Binary Classification** | Accuracy, F1, AUC-ROC | Precision, Recall |
| **Multi-class Classification** | Accuracy, Macro F1 | Per-class metrics, confusion matrix |
| **Regression** | MSE, RMSE, MAE | R², MAPE |
| **Object Detection** | mAP | Precision, Recall at IoU threshold |
| **Segmentation** | IoU, Dice | Pixel accuracy |
| **RL** | Cumulative reward | Episode length, success rate |

### Evaluation Best Practices

- Use proper train/val/test splits
- Cross-validation for small datasets
- Stratified splits for imbalanced data
- Time-based splits for temporal data
- Domain-based splits for generalization

---

## 8. Deployment Patterns

### Deployment Options

| Pattern | Use Cases | Considerations |
|---------|-----------|----------------|
| **REST API** | General serving | Simple, stateless, HTTP overhead |
| **gRPC** | High-performance serving | Binary, efficient, requires client support |
| **Batch Processing** | Offline inference | High throughput, latency acceptable |
| **Edge Deployment** | Mobile, IoT | Model compression, optimization needed |
| **Streaming** | Real-time data | Kafka, message queues |

### Model Optimization

| Technique | Purpose | Trade-offs |
|-----------|---------|------------|
| **Quantization** | Reduce model size | Slight accuracy loss |
| **Pruning** | Remove unused weights | Complexity, accuracy impact |
| **Distillation** | Smaller student model | Training overhead |
| **ONNX** | Cross-framework deployment | Compatibility |

---

## 9. MLOps Principles

### Model Lifecycle

1. **Experimentation** - Track experiments, hyperparameters
2. **Training** - Reproducible pipelines, version control
3. **Validation** - Comprehensive evaluation, A/B testing
4. **Deployment** - Gradual rollout, monitoring
5. **Monitoring** - Performance tracking, drift detection
6. **Retraining** - Automated triggers, data updates

### Key Tools

| Category | Tools |
|----------|-------|
| **Experiment Tracking** | MLflow, Weights & Biases, Neptune |
| **Data Versioning** | DVC, Pachyderm |
| **Model Registry** | MLflow, SageMaker |
| **Serving** | TF Serving, TorchServe, KServe |
| **Orchestration** | Kubeflow, Airflow |
| **Monitoring** | Prometheus, Grafana, Evidently |

---

## 10. Common Pitfalls

| Problem | Solution |
|---------|----------|
| **Data leakage** | Careful train/test split, feature engineering after split |
| **Overfitting** | Regularization, more data, simpler model |
| **Imbalanced data** | Class weights, resampling, proper metrics |
| **Vanishing gradients** | Batch norm, ResNet, careful initialization |
| **Exploding gradients** | Gradient clipping, proper learning rate |
| **Poor generalization** | Data augmentation, cross-validation, domain adaptation |

---

> **Remember:** Start simple, measure everything, iterate based on data. The best model is the one that solves the problem, not the most complex one.
