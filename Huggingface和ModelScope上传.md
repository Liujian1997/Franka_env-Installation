# Huggingface和ModelScope上传

- [Huggingface和ModelScope上传](#huggingface和modelscope上传)
  - [Huggingface上传](#huggingface上传)
  - [ModelScope上传](#modelscope上传)
  - [添加 model card 等信息](#添加-model-card-等信息)

---

## Huggingface上传

> [!TIP]
> Huggingface 推荐使用 `huggingface-cli` 进行上传，Huggingface可能会弃用 `git-lfs`，Referecnce: [上传参考](https://hugging-face.cn/docs/huggingface_hub/guides/upload#tips-and-tricks-for-large-uploads)和[Git 与 HTTP 范式区别](https://hugging-face.cn/docs/huggingface_hub/concepts/git_vs_http)

创建Repo过于简单，略过

- CLI上传
  
```shell
pip install huggingface-cli
```

在设置中，`settings->access token->generate access token`，生成一个token，用于上传。

```shell
echo 'export HF_TOKEN="hf_xxxxxxxxxxxxxxxxxxx' >> ~/.bashrc
source ~/.bashrc
```

登陆 `huggingface-cli`

```shell
huggingface-cli login
huggingface-cli upload Username/RepoName path/to/your/model --repo-type model or dataset
# 如果是大文件
huggingface-cli upload_large_folder Username/RepoName path/to/your/model --repo-type model or dataset
```

## ModelScope上传

> [!TIP]
> ModelScope 上传速度还不错，9.4Mb/s
> 创建Repo后，在 `README.md` 有写如何下载

请确保 lfs 已经被正确安装

```shell
git lfs install
git clone https://oauth2:xxxxxxxxxxxxxxxxxxx@www.modelscope.cn/datasets/UserName/RepoName.git
```

下载完成后，将模型文件放入 `RepoName` 文件夹中，然后上传

对于大文件（大小 > 100M）

使用以下命令跟踪大文件：

```shell
git lfs track <文件名>
```

然后提交变更并推送：

```shell
git add .
git commit -m "Add model weights"
git push
```


## 添加 model card 等信息

model card 信息统一在 `READMD.md` 中配置，README 在 huggingface.co 网站上可以很方便地编辑。官方提供了 metadata UI 界面，可以轻松的配置模型的一些信息。

添加的 yaml 配置示例：

```yaml
---
license: apache-2.0
language:
- en
pipeline_tag: text2text-generation
library_name: transformers
tags:
- text-generation-inference
widget:
- text: >
    Given a SQL table named 'price_data' with the following columns:
    
    Transaction_ID, Platform, Product_ID, User_ID, Transaction_Amount
    
    Construct a SQL query to answer the following question:
    
    Q: How many rows are there

  example_title: "How many rows are there?"
---
```
