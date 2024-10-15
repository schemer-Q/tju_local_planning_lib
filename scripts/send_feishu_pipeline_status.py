"""
给飞书发送pipeline状态
"""

import requests
import json
import argparse

FEISHU_WEBHOOK = "https://open.feishu.cn/open-apis/bot/v2/hook/4049a242-eb48-48e5-91d4-4c838836d67b"

def send_feishu_message(pipeline_status):
  """
  Args:
    pipeline_status: str success or failed or others
  """
  print(f"pipeline_status: {pipeline_status}")
  status_text = "其他"
  if pipeline_status == "success":
    status_text = "成功"
  elif pipeline_status == "failed":
    status_text = "失败"

  print(f"飞书消息: {status_text}")
  message = {
    "msg_type": "text",
    "content": {
      "text": f"TRUNK_PERCEPTION CI 完成:\n状态: {status_text}"
    }
  }
  
  response = requests.post(FEISHU_WEBHOOK, json=message)  
  print(f"飞书消息发送结果: {response.status_code}")


if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument("--pipeline_status", type=str, default="success")
  args = parser.parse_args()

  send_feishu_message(args.pipeline_status)
  
  