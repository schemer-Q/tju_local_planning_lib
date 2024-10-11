# 车道线后处理技术路线

```mermaid
graph LR
    A[Input] --> B[Predict]
    B --> C[Associate]
    C --> D[Update]
    D --> E[Lifetime Update]
    E --> F[Position Process]
    F --> G[Output]
```
