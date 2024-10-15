"""
将cppcheck结果cppcheck-report.xml解析后发送飞书
"""

import requests
import json
import xml.etree.ElementTree as ET
from collections import Counter
import argparse

FEISHU_WEBHOOK = "https://open.feishu.cn/open-apis/bot/v2/hook/4049a242-eb48-48e5-91d4-4c838836d67b"

def count_error_severities(xml_file):
    # 解析XML文件
    tree = ET.parse(xml_file)
    root = tree.getroot()
    
    # 使用Counter统计不同severity的数量
    severities = Counter(error.attrib['severity'] for error in root.findall('.//error'))
    
    return dict(severities)


def send_feishu_message(severity_counts):
    # 构建飞书消息内容
    if severity_counts: 
        message = {
            "msg_type": "text",
            "content": {
            "text": f"TRUNK_PERCEPTION CPPCHECK 任务完成:\n" + 
                        "\n".join([f"{severity}: {count}" for severity, count in severity_counts.items()])
            }
        }
    else:
        message = {
            "msg_type": "text",
            "content": {
                "text": "TRUNK_PERCEPTION CPPCHECK 任务完成:\n" +
                        "没有发现错误"
            }
        }
    print(f"飞书消息: {message}")

    # 发送POST请求到飞书API
    response = requests.post(FEISHU_WEBHOOK, json=message)
    print(f"飞书消息发送结果: {response.status_code}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--xml_file", type=str, default="cppcheck-report.xml")
    args = parser.parse_args()


    severity_counts = count_error_severities(args.xml_file)
    print(f"cppcheck结果: {severity_counts}")

    # 发送飞书消息
    send_feishu_message(severity_counts)









