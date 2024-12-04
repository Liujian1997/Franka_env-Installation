# Google Drive命令行下载

- [Google Drive命令行下载](#google-drive命令行下载)
  - [通过gdown下载](#通过gdown下载)
    - [安装gdown](#安装gdown)
    - [Usage](#usage)
      - [via Command Line](#via-command-line)
      - [via Python](#via-python)
  - [不用挂梯子的方法](#不用挂梯子的方法)
    - [注册multcloud](#注册multcloud)

---

## 通过gdown下载

### 安装gdown

```shell
pip install gdown
```

### Usage

> [!TIPS]
> [Reference Repository](https://github.com/wkentaro/gdown)

#### via Command Line

```shell
$ gdown --help
usage: gdown [-h] [-V] [-O OUTPUT] [-q] [--fuzzy] [--id] [--proxy PROXY]
             [--speed SPEED] [--no-cookies] [--no-check-certificate]
             [--continue] [--folder] [--remaining-ok]
             url_or_id
...

$ # a large file (~500MB)
$ gdown https://drive.google.com/uc?id=1l_5RK28JRL19wpT22B-DY9We3TVXnnQQ
$ md5sum fcn8s_from_caffe.npz
256c2a8235c1c65e62e48d3284fbd384

$ # same as the above but with the file ID
$ gdown 1l_5RK28JRL19wpT22B-DY9We3TVXnnQQ

$ # a small file
$ gdown https://drive.google.com/uc?id=0B9P1L--7Wd2vU3VUVlFnbTgtS2c
$ cat spam.txt
spam

$ # download with fuzzy extraction of a file ID
$ gdown --fuzzy 'https://drive.google.com/file/d/0B9P1L--7Wd2vU3VUVlFnbTgtS2c/view?usp=sharing&resourcekey=0-WWs_XOSctfaY_0-sJBKRSQ'
$ cat spam.txt
spam

$ # --fuzzy option also works with Microsoft Powerpoint files
$ gdown --fuzzy "https://docs.google.com/presentation/d/15umvZKlsJ3094HNg5S4vJsIhxcFlyTeK/edit?usp=sharing&ouid=117512221203072002113&rtpof=true&sd=true"

$ # a folder
$ gdown https://drive.google.com/drive/folders/15uNXeRBIhVvZJIhL4yTw4IsStMhUaaxl -O /tmp/folder --folder

## [!NOTE] 链接后面必须跟参数?resourcekey=0-4V4AQmv6RCytSv0_ua2HKg
## 具体如下：
## https://drive.google.com/drive/folders/0B2LlLwoO3nfZfnJnSmNHTXZycUtFN0k3Y3o2SjlvWUg1WHRkU1Y2OE1wdkN6V1BkRGF6MG8?resourcekey=0-4V4AQmv6RCytSv0_ua2HKg

$ # as an alternative to curl/wget
$ gdown https://httpbin.org/ip -O ip.json
$ cat ip.json
{
  "origin": "126.169.213.247"
}

$ # write stdout and pipe to extract
$ gdown https://github.com/wkentaro/gdown/archive/refs/tags/v4.0.0.tar.gz -O - --quiet | tar zxvf -
$ ls gdown-4.0.0/
gdown  github2pypi  LICENSE  MANIFEST.in  pyproject.toml  README.md  setup.cfg  setup.py  tests
```

#### via Python

```python
import gdown

# a file
url = "https://drive.google.com/uc?id=1l_5RK28JRL19wpT22B-DY9We3TVXnnQQ"
output = "fcn8s_from_caffe.npz"
gdown.download(url, output)

# same as the above, but with the file ID
id = "0B9P1L--7Wd2vNm9zMTJWOGxobkU"
gdown.download(id=id, output=output)

# same as the above, and you can copy-and-paste a URL from Google Drive with fuzzy=True
url = "https://drive.google.com/file/d/0B9P1L--7Wd2vNm9zMTJWOGxobkU/view?usp=sharing"
gdown.download(url=url, output=output, fuzzy=True)

# Cached download with identity check via MD5 (or SHA1, SHA256, etc).
# Pass postprocess function e.g., extracting compressed file.
md5 = "md5:fa837a88f0c40c513d975104edf3da17"
gdown.cached_download(url, output, hash=hash, postprocess=gdown.extractall)

# a folder
url = "https://drive.google.com/drive/folders/15uNXeRBIhVvZJIhL4yTw4IsStMhUaaxl"
gdown.download_folder(url)

# same as the above, but with the folder ID
id = "15uNXeRBIhVvZJIhL4yTw4IsStMhUaaxl"
gdown.download_folder(id=id)
```

---

## 不用挂梯子的方法

### 注册[multcloud](https://www.multcloud.com/)

这是一个可以管理多个云盘并可以跨平台转存文件的云盘管理工具

将Google drive账号添加到Multi cloud。其中添加Google drive需要梯子，添加完成之后就不用了
