#!/usr/bin/env python3
# coding=utf-8

import rospy
from dotenv import load_dotenv
import os
from os.path import join, dirname

dotenv_path = join(dirname(__file__), '.env')
load_dotenv(dotenv_path)
AP = os.environ.get("API_KEY") # 環境変数の値をAPに代入
print(AP)