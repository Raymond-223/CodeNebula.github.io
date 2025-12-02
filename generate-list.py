#!/usr/bin/env python3
"""
CodeNebula 文件列表生成器
在本地运行此脚本，自动生成 file-list.json
"""

import os
import json
import sys

def main():
    print("=" * 50)
    print("CodeNebula 文件列表生成器")
    print("=" * 50)
    
    # 检查 algorithm 文件夹是否存在
    if not os.path.exists("algorithm"):
        print("❌ 错误: algorithm 文件夹不存在！")
        print("请在当前目录创建 algorithm 文件夹")
        return
    
    print("正在扫描 algorithm 文件夹...")
    
    structure = {}
    
    # 遍历 algorithm 文件夹
    for item in os.listdir("algorithm"):
        item_path = os.path.join("algorithm", item)
        
        if os.path.isdir(item_path):
            print(f"📁 发现文件夹: {item}")
            files = []
            
            # 扫描文件夹中的 MD 文件
            for file_item in os.listdir(item_path):
                if file_item.lower().endswith('.md'):
                    file_info = {
                        "name": file_item,
                        "display_name": file_item.replace('.md', '').replace('.MD', ''),
                        "path": f"algorithm/{item}/{file_item}",
                        "url": f"https://raw.githubusercontent.com/raymond-223/CodeNebula.github.io/main/algorithm/{item}/{file_item}"
                    }
                    files.append(file_info)
                    print(f"   📄 {file_item}")
            
            if files:
                # 按文件名排序
                files.sort(key=lambda x: x["name"].lower())
                structure[item] = {
                    "name": item,
                    "files": files
                }
    
    # 保存为 JSON 文件
    output_file = "algorithm/file-list.json"
    with open(output_file, "w", encoding="utf-8") as f:
        json.dump(structure, f, ensure_ascii=False, indent=2)
    
    print("\n" + "=" * 50)
    print(f"✅ 生成成功！")
    print(f"📁 文件夹数量: {len(structure)}")
    
    total_files = sum(len(folder["files"]) for folder in structure.values())
    print(f"📄 文件总数: {total_files}")
    print(f"💾 保存位置: {output_file}")
    print("\n🔧 下一步:")
    print("1. 将 file-list.json 上传到 GitHub")
    print("2. 将更新后的 algorithm.html 上传到 GitHub")
    print("3. 访问: https://raymond-223.github.io/CodeNebula.github.io/algorithm.html")
    print("=" * 50)

if __name__ == "__main__":
    main()
