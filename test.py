import requests

# 定义请求的 URL
url = 'http://192.168.1.220:80'  # 请替换为实际的 URL

# 定义要发送的数据（字典形式）
data = {
    'command': True,
}

# 发送 POST 请求
response = requests.post(url, data=data)

# 打印响应内容
print(response.text)