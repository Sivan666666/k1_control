import sys
import os

print("--- Python Executable ---")
print(sys.executable)
print("\n--- sys.path ---")
# 使用 pprint 模块让输出更整洁
from pprint import pprint
pprint(sys.path)

print(f"\n--- User home directory (os.path.expanduser('~')) ---")
print(os.path.expanduser('~'))