#!/bin/bash
set -e

help_text="""[Usage]: bash install_dep.sh [options]
default is to install the debs enabled, same as -i
-i : install the debs enabled
-d </path/to/save/deb> : download the debs enabled
-r : remove the debs enabled
-h : show help message
"""

action="install"
download_dir=

while getopts "ird:h" opt; do
case $opt in
    i)
    action="install"
    echo "will install the debs enabled."
    ;;
    d)
    action="download"
    download_dir=$OPTARG
    echo "will down load debs to: $OPTARG"
    ;;
    r)
    action="remove"
    echo "will remove the debs enabled"
    ;;
    h)
    action=
    echo "$help_text"
    exit 0
    ;;
    \?)
    action=
    echo "Invalid option: -$OPTARG"
    exit 1
    ;;
esac
done

config_path=config.json
echo $config_path
if [ ! -f $config_path ];then
    echo "config_path is not exit, please check"
    exit 1
fi

# 判断是否安装jq，并提示安装方法
which jq
if [ $? -ne 0 ];then
    echo -e "\033[41;37m cannot find jq, please use \"sudo apt install jq\" to install \033[0m"
    exit 1
fi

jq_search_key=project.dependencies

# 处理 dev 和 sdk 依赖
for dep_type in "dev" "sdk"; do
    dependencies=`jq -r ".project.dependencies.$dep_type" $config_path`
    deb_names=`echo $dependencies | jq -r 'keys[]'`

    echo "处理 $dep_type 依赖："

    # 对使能了的deb包: 安装、下载、卸载
    for deb_name in $deb_names; do
        deb_dict=`echo $dependencies | jq ".$deb_name"`
        deb_enable=`echo $deb_dict | jq -r '.enable'`
        deb_version=`echo $deb_dict | jq -r '.version'`
        if [ "$deb_enable" = "true" ]; then
            echo "============ 需要 $deb_name ==============="
            if [ "$action" = "install" ]; then
                apt install $deb_name=$deb_version
            elif [ "$action" = "download" ]; then
                pushd $download_dir
                apt-get download $deb_name=$deb_version
                popd
            elif [ "$action" = "remove" ]; then
                apt remove $deb_name=$deb_version -y
            else 
                echo "不支持的操作：$action"
            fi
        fi
    done
done
