#!/bin/bash

# CSDN图片链接列表
declare -a urls=(
    "https://i-blog.csdnimg.cn/direct/19408dcab57446e5adb8bc8154f9181e.png"
    "https://i-blog.csdnimg.cn/direct/02e7bb7760c54764bc56e9b739c12c33.jpeg"
    "https://i-blog.csdnimg.cn/direct/4d58b5445deb43ac8b717d46b451aed7.png"
    "https://i-blog.csdnimg.cn/direct/111f0648c38649e0b3feab1969539d32.png"
    "https://i-blog.csdnimg.cn/direct/dc100f30dc6e43348a6a93c7f2896099.png"
    "https://i-blog.csdnimg.cn/direct/705573d2c01b4fbcbcc1ef96ab11cd79.png"
    "https://i-blog.csdnimg.cn/direct/8540ead19ce8487ca6d85f50ee3a7e55.png"
    "https://i-blog.csdnimg.cn/direct/8259823340a44c088744758a7b567148.png"
)

# 起始编号（从66开始，因为现有图片到65.png）
start_num=66

echo "开始下载CSDN图片到images目录..."

for i in "${!urls[@]}"; do
    url="${urls[$i]}"
    file_num=$((start_num + i))
    
    # 根据URL扩展名确定文件扩展名
    if [[ $url == *.jpeg ]] || [[ $url == *.jpg ]]; then
        extension="jpeg"
    else
        extension="png"
    fi
    
    filename="images/${file_num}.${extension}"
    
    echo "下载: $url -> $filename"
    
    # 使用curl下载图片
    curl -s -L "$url" -o "$filename"
    
    if [ $? -eq 0 ]; then
        echo "  成功下载: $filename"
    else
        echo "  下载失败: $url"
    fi
    
    # 添加延迟避免请求过快
    sleep 1
done

echo "图片下载完成！"