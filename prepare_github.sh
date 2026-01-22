#!/bin/bash
# GitHub上传准备脚本

echo "================================================"
echo "  FAIRINO FR5 视觉伺服项目 - GitHub上传准备"
echo "================================================"

# 1. 备份个人数据
echo ""
echo "[1/6] 备份个人标定数据..."
mkdir -p ~/backup_d435i_fr5_$(date +%Y%m%d)
cp -v handeye_samples.npz ~/backup_d435i_fr5_$(date +%Y%m%d)/ 2>/dev/null || true
cp -v handeye_result.txt ~/backup_d435i_fr5_$(date +%Y%m%d)/ 2>/dev/null || true
cp -v config.yaml ~/backup_d435i_fr5_$(date +%Y%m%d)/ 2>/dev/null || true
cp -v tool_frame_params.txt ~/backup_d435i_fr5_$(date +%Y%m%d)/ 2>/dev/null || true
echo "✓ 数据已备份到: ~/backup_d435i_fr5_$(date +%Y%m%d)/"

# 2. 清理临时文件
echo ""
echo "[2/6] 清理临时文件..."
rm -vf test1.py 2>/dev/null || true
rm -vf test.py 2>/dev/null || true
rm -vf aruco_2d_visual_servo_direction.md 2>/dev/null || true
echo "✓ 临时文件已清理"

# 3. 检查必需文件
echo ""
echo "[3/6] 检查核心文件..."
required_files=(
    "main_vs.py"
    "fr5_driver.py"
    "aruco_detector.py"
    "controller.py"
    "handeye_calibration_eye_in_hand.py"
    "handeye_io.py"
    "utils_math.py"
    "requirements.txt"
    "config.yaml.example"
    ".gitignore"
    "README.md"
)

missing=0
for file in "${required_files[@]}"; do
    if [ -f "$file" ]; then
        echo "  ✓ $file"
    else
        echo "  ✗ $file (缺失)"
        missing=$((missing+1))
    fi
done

if [ $missing -gt 0 ]; then
    echo "⚠ 警告：有 $missing 个必需文件缺失！"
else
    echo "✓ 所有核心文件就绪"
fi

# 4. 显示即将被忽略的文件
echo ""
echo "[4/6] 检查.gitignore规则..."
if [ -f .gitignore ]; then
    echo "✓ .gitignore 已存在"
    echo ""
    echo "以下文件/文件夹将被忽略（不会上传到GitHub）："
    echo "  • __pycache__/"
    echo "  • .venv/"
    echo "  • handeye_logs/"
    echo "  • servo_logs/"
    echo "  • config.yaml (个人配置)"
    echo "  • handeye_samples.npz (个人标定数据)"
    echo "  • handeye_result.txt (个人标定结果)"
    echo "  • fairino/ (第三方SDK)"
    echo "  • lib/ include/ (第三方库)"
else
    echo "⚠ 警告：.gitignore 不存在！"
fi

# 5. 统计文件
echo ""
echo "[5/6] 文件统计..."
echo "Python源文件: $(find . -maxdepth 1 -name "*.py" -type f | wc -l) 个"
echo "配置文件: $(find . -maxdepth 1 -name "*.yaml*" -type f | wc -l) 个"
echo "文档文件: $(find . -maxdepth 1 -name "*.md" -type f | wc -l) 个"

# 6. Git状态
echo ""
echo "[6/6] Git仓库状态..."
if [ -d .git ]; then
    echo "✓ Git仓库已初始化"
    echo ""
    echo "当前分支:"
    git branch --show-current 2>/dev/null || echo "  (未设置分支)"
    echo ""
    echo "远程仓库:"
    git remote -v 2>/dev/null || echo "  (未配置远程仓库)"
else
    echo "⚠ Git仓库未初始化"
fi

echo ""
echo "================================================"
echo "  准备完成！"
echo "================================================"
echo ""
echo "下一步操作："
echo ""
echo "1. 初始化Git仓库（如果未初始化）："
echo "   git init"
echo ""
echo "2. 添加所有文件："
echo "   git add -A"
echo ""
echo "3. 查看将要提交的文件："
echo "   git status"
echo ""
echo "4. 提交："
echo "   git commit -m 'Initial commit: FAIRINO FR5 Visual Servo System'"
echo ""
echo "5. 在GitHub创建仓库后，添加远程地址："
echo "   git remote add origin https://github.com/YOUR_USERNAME/REPO_NAME.git"
echo ""
echo "6. 推送到GitHub："
echo "   git branch -M main"
echo "   git push -u origin main"
echo ""
echo "================================================"
