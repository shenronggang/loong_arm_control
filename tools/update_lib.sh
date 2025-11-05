#!/bin/bash/*
# Copyright [2025] [国家地方共建人形机器人创新中心/人形机器人（上海）有限公司, https://www.openloong.net/]
# Thanks for the open biped control project Nabo: <https://github.com/tryingfly/nabo>

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#         http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# ============ ***doc description @ yyp*** ============
# 自动拷贝，需安装：
# sudo apt install expect
#======================================================



# echo update_lib.sh 首次运行，务必修改下方dir路径，匹配机器人中个人项目文件夹！然后注释本两行以继续执行！
# exit

#务必修改dir！否则会覆盖别人的运行环境！
dir="~/yyp/nabo/"


if [ ${#1} = 0 ]; then
    echo ==!!update_lib.sh 参数不匹配!!==
    echo x: x64
    echo a: aarch64
    echo =================
    exit
fi

if [ '$1' == 'x' ]; then
    lib=libnabo_manipulation_x64.so
else
    lib=libnabo_manipulation_a64.so
fi

sshCopy(){
    expect <(cat <<EOF
    set timeout 1
    spawn scp ../nabo_output/$lib enpht@192.168.1.$1:$dir
    expect "password:" {
        send "1\n"
        interact
        puts "已更新文件{$1}号机"
    }
EOF
)
}

cat <(sshCopy 201) &
cat <(sshCopy 202) &
cat <(sshCopy 203) &
cat <(sshCopy 204) &
cat <(sshCopy 205) &
cat <(sshCopy 206) &
cat <(sshCopy 207)
sleep 1

