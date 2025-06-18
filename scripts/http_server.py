#!/usr/bin/env python3
import http.server
import socketserver
import os

# ポート番号の設定
PORT = 8080

# 公開するディレクトリを指定
DIRECTORY = "/home/nakaba/catkin_ws/src/tc2024/gui"

# カレントディレクトリを公開ディレクトリに変更
os.chdir(DIRECTORY)

# HTTPハンドラー
Handler = http.server.SimpleHTTPRequestHandler

# サーバの起動
with socketserver.TCPServer(("", PORT), Handler) as httpd:
    print(f"Serving at port {PORT}")
    print(f"Serving directory: {DIRECTORY}")
    httpd.serve_forever()
