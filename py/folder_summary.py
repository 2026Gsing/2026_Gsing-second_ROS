#!/usr/bin/env python3
"""
自动生成项目结构和内容报告
使用图形化界面选择目录
用法: python3 generate_report.py
"""

import os
import sys
from pathlib import Path
import fnmatch
from datetime import datetime
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
import threading

# 配置 - 只排除不需要的文件
EXCLUDE_DIRS = [
    'build', 'install', 'log', '__pycache__', 
    '.git', '.vscode', '.idea', 'venv', 'env',
    '.pytest_cache', '.coverage', '*.egg-info',
    'devel', 'logs'
]

EXCLUDE_FILES = [
    '*.pyc', '*.pyo', '*.so', '*.dll', '*.dylib',
    '*.png', '*.jpg', '*.jpeg', '*.gif', '*.bmp',
    '*.zip', '*.tar', '*.gz', '*.bz2', '*.7z'
]

# 所有可能有用的文件类型
IMPORTANT_FILES = [
    # ROS/机器人相关
    '*.launch.py', '*.launch.xml', '*.launch', '*.xacro', '*.urdf', 
    '*.world', '*.sdf', '*.yaml', '*.yml', '*.xml', 
    'package.xml', 'CMakeLists.txt', 'setup.py', 'setup.cfg',
    
    # 代码文件
    '*.py', '*.cpp', '*.hpp', '*.c', '*.h', '*.cc', '*.cxx',
    '*.java', '*.js', '*.ts', '*.go', '*.rs', '*.rb',
    '*.php', '*.swift', '*.kt', '*.scala',
    
    # 脚本文件
    '*.sh', '*.bash', '*.zsh', '*.fish', '*.ps1', '*.bat',
    '*.cmake', '*.mk', 'Makefile', '*.make',
    
    # 配置文件
    '*.cfg', '*.conf', '*.config', '*.ini', '*.env',
    '*.json', '*.toml', '.gitignore', '.dockerignore',
    '*.dockerfile', 'Dockerfile', '*.container',
    
    # 文档文件
    'README.md', 'README.rst', 'README.txt', 'README',
    '*.md', '*.rst', '*.txt',
    'CHANGELOG.md', 'CONTRIBUTING.md', 'LICENSE',
    
    # 数据文件
    '*.csv', '*.tsv', '*.jsonl', '*.sql', '*.db',
    '*.sqlite', '*.hdf5', '*.h5', '*.npy', '*.npz',
    
    # 网格/模型文件
    '*.stl', '*.obj', '*.dae', '*.ply', '*.3mf',
    '*.step', '*.stp', '*.iges', '*.igs',
    
    # 仿真相关
    '*.gazebo', '*.gazebo.xacro', '*.gazebo.urdf',
    '*.rviz', '*.rqt', '*.sdf',
    
    # 参数文件
    '*.params', '*.param',
    
    # 工作空间配置
    '*.workspace', '*.code-workspace',
    '.vscode/*.json', '.idea/*.xml',
    
    # 依赖文件
    'requirements.txt', 'requirements-dev.txt',
    'Pipfile', 'Pipfile.lock', 'poetry.lock',
    'pyproject.toml', 'setup.py', 'setup.cfg',
    'package.json', 'package-lock.json',
    'Cargo.toml', 'Cargo.lock', 'go.mod', 'go.sum',
    'Gemfile', 'Gemfile.lock',
    
    # Docker相关
    'docker-compose.yml', 'docker-compose.yaml',
    'docker-compose.*.yml', 'Dockerfile.*',
    
    # CI/CD
    '.github/**/*.yml', '.gitlab-ci.yml',
    '.travis.yml', 'Jenkinsfile', '.circleci/**/*.yml',
    
    # 编译相关
    '*.cmake', '*.mk', '*.am', '*.ac',
    
    # 测试相关
    'test_*.py', '*_test.py', '*.test',
    '*.spec.js', '*.test.js',
    
    # 网格地图
    '*.pgm',  # 用于ROS地图
    '*.bt', '*.btconf',  # 用于Octomap
    
    # URDF/机器人描述
    '*.urdf.xacro', '*.macro.xacro',
    
    # 机器人运动学/动力学
    '*.srdf', '*.srdf.xacro',
    
    # 机器人配置文件
    '*.controllers.yaml', '*.hardware.yaml',
    
    # 运动规划
    '*.ompl', '*.ompl.yaml',
    
    # 导航相关
    '*.costmap.yaml', '*.planner.yaml',
    '*.local.yaml', '*.global.yaml',
    
    # 传感器配置
    '*.sensor.yaml', '*.lidar.yaml', '*.camera.yaml',
    '*.imu.yaml', '*.gps.yaml',
    
    # 控制参数
    '*.control.yaml', '*.gains.yaml',
    '*.pid.yaml', '*.controller.yaml',
    
    # 接口定义
    '*.msg', '*.srv', '*.action',  # ROS接口
    '*.proto',  # Protobuf
    '*.thrift',  # Thrift
    '*.idl',  # IDL
    
    # 建模文件
    '*.mod', '*.model', '*.model.yaml',
    
    # 材质/纹理
    '*.mtl', '*.material', '*.material.xacro',
    
    # 场景文件
    '*.scene', '*.scene.yaml', '*.world.yaml',
    
    # 仿真配置
    '*.gazebo.yaml', '*.sim.yaml',
    
    # 任务描述
    '*.task', '*.mission', '*.behavior',
]

class ReportGeneratorGUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("项目文件内容提取器")
        self.root.geometry("600x450")
        self.root.resizable(True, True)
        
        # 设置样式
        self.root.configure(bg='#f0f0f0')
        
        # 变量
        self.selected_dir = tk.StringVar()
        self.report_content = None
        self.report_file = None
        
        # 创建UI
        self.create_widgets()
        
        # 居中显示
        self.center_window()
        
    def center_window(self):
        """窗口居中"""
        self.root.update_idletasks()
        width = self.root.winfo_width()
        height = self.root.winfo_height()
        x = (self.root.winfo_screenwidth() // 2) - (width // 2)
        y = (self.root.winfo_screenheight() // 2) - (height // 2)
        self.root.geometry(f'{width}x{height}+{x}+{y}')
    
    def create_widgets(self):
        """创建UI组件"""
        # 标题
        title_frame = tk.Frame(self.root, bg='#2c3e50', height=60)
        title_frame.pack(fill=tk.X)
        title_frame.pack_propagate(False)
        
        title_label = tk.Label(
            title_frame, 
            text="📋 项目文件内容提取器", 
            bg='#2c3e50', 
            fg='white',
            font=('Arial', 16, 'bold')
        )
        title_label.pack(expand=True)
        
        # 主内容区域
        main_frame = tk.Frame(self.root, bg='#f0f0f0', padx=20, pady=20)
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # 目录选择区域
        select_frame = tk.LabelFrame(
            main_frame, 
            text="📁 选择要分析的目录", 
            bg='#f0f0f0',
            font=('Arial', 10, 'bold'),
            padx=10, 
            pady=10
        )
        select_frame.pack(fill=tk.X, pady=(0, 10))
        
        # 路径显示和浏览按钮
        path_frame = tk.Frame(select_frame, bg='#f0f0f0')
        path_frame.pack(fill=tk.X)
        
        self.path_entry = tk.Entry(
            path_frame, 
            textvariable=self.selected_dir,
            width=50,
            state='readonly',
            bg='white',
            font=('Arial', 9)
        )
        self.path_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 10))
        
        browse_btn = tk.Button(
            path_frame,
            text="浏览...",
            command=self.browse_directory,
            bg='#3498db',
            fg='white',
            font=('Arial', 9, 'bold'),
            padx=15,
            cursor='hand2'
        )
        browse_btn.pack(side=tk.RIGHT)
        
        # 快速选择按钮
        quick_frame = tk.Frame(select_frame, bg='#f0f0f0')
        quick_frame.pack(fill=tk.X, pady=(10, 0))
        
        quick_label = tk.Label(
            quick_frame, 
            text="快速选择:",
            bg='#f0f0f0',
            font=('Arial', 9)
        )
        quick_label.pack(side=tk.LEFT, padx=(0, 10))
        
        # 常用目录按钮
        quick_buttons = [
            ("当前目录", Path.cwd()),
            ("主目录", Path.home()),
            ("脚本目录", Path(__file__).parent.absolute()),
            ("上级目录", Path.cwd().parent),
        ]
        
        for name, path in quick_buttons:
            btn = tk.Button(
                quick_frame,
                text=name,
                command=lambda p=path: self.set_quick_dir(p),
                bg='#95a5a6',
                fg='white',
                font=('Arial', 8),
                padx=10,
                cursor='hand2'
            )
            btn.pack(side=tk.LEFT, padx=2)
        
        # 简单选项
        options_frame = tk.LabelFrame(
            main_frame,
            text="⚙️ 选项",
            bg='#f0f0f0',
            font=('Arial', 10, 'bold'),
            padx=10,
            pady=10
        )
        options_frame.pack(fill=tk.X, pady=(0, 10))
        
        # 选项
        row = tk.Frame(options_frame, bg='#f0f0f0')
        row.pack(fill=tk.X, pady=2)
        
        self.include_tree = tk.BooleanVar(value=True)
        tree_check = tk.Checkbutton(
            row,
            text="显示文件树",
            variable=self.include_tree,
            bg='#f0f0f0',
            font=('Arial', 9)
        )
        tree_check.pack(side=tk.LEFT, padx=5)
        
        self.include_hidden = tk.BooleanVar(value=False)
        hidden_check = tk.Checkbutton(
            row,
            text="包含隐藏文件",
            variable=self.include_hidden,
            bg='#f0f0f0',
            font=('Arial', 9)
        )
        hidden_check.pack(side=tk.LEFT, padx=5)
        
        # 说明文字
        info_label = tk.Label(
            options_frame,
            text="注：将显示所有找到文件的完整内容",
            bg='#f0f0f0',
            font=('Arial', 8, 'italic'),
            fg='#666666'
        )
        info_label.pack(anchor=tk.W, padx=5, pady=(5, 0))
        
        # 进度条区域
        self.progress_frame = tk.Frame(main_frame, bg='#f0f0f0')
        self.progress_frame.pack(fill=tk.X, pady=(0, 10))
        
        self.progress_bar = ttk.Progressbar(
            self.progress_frame,
            mode='indeterminate',
            length=400
        )
        
        self.progress_label = tk.Label(
            self.progress_frame,
            text="",
            bg='#f0f0f0',
            font=('Arial', 9)
        )
        
        # 按钮区域
        button_frame = tk.Frame(main_frame, bg='#f0f0f0')
        button_frame.pack(fill=tk.X)
        
        self.generate_btn = tk.Button(
            button_frame,
            text="🚀 提取全部内容",
            command=self.generate_report,
            bg='#27ae60',
            fg='white',
            font=('Arial', 11, 'bold'),
            padx=30,
            pady=10,
            cursor='hand2',
            state='normal'
        )
        self.generate_btn.pack(side=tk.LEFT, padx=5)
        
        self.open_btn = tk.Button(
            button_frame,
            text="📄 查看结果",
            command=self.open_report,
            bg='#3498db',
            fg='white',
            font=('Arial', 11, 'bold'),
            padx=30,
            pady=10,
            cursor='hand2',
            state='disabled'
        )
        self.open_btn.pack(side=tk.LEFT, padx=5)
        
        exit_btn = tk.Button(
            button_frame,
            text="❌ 退出",
            command=self.root.quit,
            bg='#e74c3c',
            fg='white',
            font=('Arial', 11, 'bold'),
            padx=30,
            pady=10,
            cursor='hand2'
        )
        exit_btn.pack(side=tk.RIGHT, padx=5)
        
        # 状态栏
        status_frame = tk.Frame(self.root, bg='#34495e', height=25)
        status_frame.pack(fill=tk.X, side=tk.BOTTOM)
        status_frame.pack_propagate(False)
        
        self.status_label = tk.Label(
            status_frame,
            text="就绪",
            bg='#34495e',
            fg='white',
            font=('Arial', 8),
            anchor=tk.W
        )
        self.status_label.pack(fill=tk.X, padx=10)
    
    def browse_directory(self):
        """打开目录浏览对话框"""
        directory = filedialog.askdirectory(
            title="选择要分析的目录",
            initialdir=self.selected_dir.get() or os.path.expanduser("~")
        )
        if directory:
            self.selected_dir.set(directory)
            self.status_label.config(text=f"已选择: {directory}")
    
    def set_quick_dir(self, path):
        """设置快速选择的目录"""
        if path:
            self.selected_dir.set(str(path))
            self.status_label.config(text=f"已选择: {path}")
    
    def generate_report(self):
        """生成报告"""
        directory = self.selected_dir.get()
        if not directory:
            messagebox.showwarning("警告", "请先选择要分析的目录！")
            return
        
        if not os.path.exists(directory):
            messagebox.showerror("错误", "选择的目录不存在！")
            return
        
        # 禁用按钮
        self.generate_btn.config(state='disabled')
        self.open_btn.config(state='disabled')
        
        # 显示进度条
        self.progress_bar.pack(pady=5)
        self.progress_label.pack()
        self.progress_bar.start(10)
        self.progress_label.config(text="正在提取全部文件内容...")
        self.status_label.config(text=f"处理中: {directory}")
        
        # 在新线程中生成报告
        thread = threading.Thread(target=self._generate_report_thread, args=(directory,))
        thread.daemon = True
        thread.start()
    
    def _generate_report_thread(self, directory):
        """在线程中生成报告"""
        try:
            # 获取用户选项
            options = {
                'include_tree': self.include_tree.get(),
                'include_hidden': self.include_hidden.get()
            }
            
            report = extract_files_content(
                Path(directory),
                options
            )
            
            # 保存报告
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            project_name = Path(directory).name or "project"
            output_file = Path(__file__).parent.absolute() / f"{project_name}_full_content_{timestamp}.txt"
            
            with open(output_file, 'w', encoding='utf-8') as f:
                f.write(report)
            
            # 更新UI
            self.root.after(0, self._report_generated, report, output_file)
            
        except Exception as e:
            self.root.after(0, self._report_error, str(e))
    
    def _report_generated(self, report, output_file):
        """报告生成完成后的UI更新"""
        self.progress_bar.stop()
        self.progress_bar.pack_forget()
        self.progress_label.pack_forget()
        
        self.generate_btn.config(state='normal')
        self.open_btn.config(state='normal')
        
        self.report_content = report
        self.report_file = output_file
        
        # 计算大概的文件数
        file_count = report.count('📄')
        
        self.status_label.config(text=f"已提取: {output_file.name} ({file_count}个文件)")
        
        # 显示成功消息
        messagebox.showinfo(
            "成功", 
            f"文件内容提取成功！\n\n"
            f"📁 分析目录: {self.selected_dir.get()}\n"
            f"📄 保存位置: {output_file}\n"
            f"📊 文件大小: {len(report)} 字符\n"
            f"📝 文件数量: {file_count} 个\n\n"
            f"点击'查看结果'浏览全部内容"
        )
    
    def _report_error(self, error_msg):
        """报告生成错误处理"""
        self.progress_bar.stop()
        self.progress_bar.pack_forget()
        self.progress_label.pack_forget()
        
        self.generate_btn.config(state='normal')
        self.status_label.config(text="提取失败")
        
        messagebox.showerror("错误", f"提取内容时发生错误:\n{error_msg}")
    
    def open_report(self):
        """打开报告"""
        if self.report_content and self.report_file:
            # 创建新窗口显示报告
            report_window = tk.Toplevel(self.root)
            report_window.title(f"文件内容 - {self.report_file.name}")
            report_window.geometry("900x700")
            
            # 创建文本框和滚动条
            frame = tk.Frame(report_window)
            frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
            
            # 添加搜索框
            search_frame = tk.Frame(frame)
            search_frame.pack(fill=tk.X, pady=(0, 5))
            
            tk.Label(search_frame, text="搜索:").pack(side=tk.LEFT)
            search_entry = tk.Entry(search_frame, width=30)
            search_entry.pack(side=tk.LEFT, padx=5)
            
            def search_text():
                search_term = search_entry.get()
                if search_term:
                    text_widget.tag_remove('search', '1.0', tk.END)
                    start_pos = '1.0'
                    while True:
                        start_pos = text_widget.search(search_term, start_pos, stopindex=tk.END, nocase=True)
                        if not start_pos:
                            break
                        end_pos = f"{start_pos}+{len(search_term)}c"
                        text_widget.tag_add('search', start_pos, end_pos)
                        start_pos = end_pos
                    text_widget.tag_config('search', background='yellow', foreground='black')
            
            search_btn = tk.Button(search_frame, text="查找", command=search_text)
            search_btn.pack(side=tk.LEFT, padx=5)
            
            # 文本显示区域
            text_frame = tk.Frame(frame)
            text_frame.pack(fill=tk.BOTH, expand=True)
            
            text_widget = tk.Text(text_frame, wrap=tk.WORD, font=('Courier', 10))
            scrollbar_y = tk.Scrollbar(text_frame, command=text_widget.yview)
            scrollbar_x = tk.Scrollbar(frame, orient=tk.HORIZONTAL, command=text_widget.xview)
            
            text_widget.configure(
                yscrollcommand=scrollbar_y.set,
                xscrollcommand=scrollbar_x.set
            )
            
            scrollbar_y.pack(side=tk.RIGHT, fill=tk.Y)
            scrollbar_x.pack(side=tk.BOTTOM, fill=tk.X)
            text_widget.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
            
            # 插入报告内容
            text_widget.insert(tk.END, self.report_content)
            text_widget.config(state=tk.DISABLED)
            
            # 底部按钮
            btn_frame = tk.Frame(report_window)
            btn_frame.pack(fill=tk.X, padx=10, pady=(0, 10))
            
            close_btn = tk.Button(
                btn_frame,
                text="关闭",
                command=report_window.destroy,
                bg='#95a5a6',
                fg='white',
                padx=20
            )
            close_btn.pack(side=tk.RIGHT, padx=5)
    
    def run(self):
        """运行GUI"""
        self.root.mainloop()

def should_exclude(path, options):
    """检查是否应该排除"""
    name = path.name
    
    # 隐藏文件处理
    if name.startswith('.') and not options.get('include_hidden', False):
        return True
    
    # 排除目录
    if path.is_dir():
        for pattern in EXCLUDE_DIRS:
            if fnmatch.fnmatch(name, pattern):
                return True
    # 排除文件
    else:
        for pattern in EXCLUDE_FILES:
            if fnmatch.fnmatch(name, pattern):
                return True
    
    return False

def is_important_file(filepath):
    """检查是否是重要文件"""
    name = filepath.name
    for pattern in IMPORTANT_FILES:
        if pattern.startswith('*'):
            if name.endswith(pattern[1:]):
                return True
        elif pattern == name:
            return True
        # 处理带路径的模式
        elif '/' in pattern:
            if fnmatch.fnmatch(str(filepath), pattern):
                return True
    return False

def read_file_content(filepath):
    """读取完整的文件内容"""
    try:
        with open(filepath, 'r', encoding='utf-8') as f:
            content = f.read()
            return content
    except UnicodeDecodeError:
        return None
    except Exception as e:
        return f"读取失败: {e}"

def generate_tree_structure(path, options, prefix="", is_last=True):
    """生成树形结构"""
    lines = []
    path = Path(path)
    name = path.name if path.name else str(path)
    
    connector = "└── " if is_last else "├── "
    lines.append(f"{prefix}{connector}{name}{'/' if path.is_dir() else ''}")
    
    if path.is_dir():
        items = []
        try:
            for item in path.iterdir():
                if not should_exclude(item, options):
                    items.append(item)
        except PermissionError:
            return lines
        
        items.sort(key=lambda x: (not x.is_dir(), x.name.lower()))
        
        for i, item in enumerate(items):
            new_prefix = prefix + ("    " if is_last else "│   ")
            lines.extend(generate_tree_structure(
                item, options, new_prefix, i == len(items) - 1
            ))
    
    return lines

def extract_files_content(root_path, options):
    """提取所有文件内容"""
    root_path = Path(root_path).resolve()
    report_lines = []
    
    # 头部信息
    report_lines.append("=" * 100)
    report_lines.append(f"📋 项目全部文件内容")
    report_lines.append("=" * 100)
    report_lines.append(f"生成时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    report_lines.append(f"分析路径: {root_path}")
    report_lines.append(f"项目名称: {root_path.name}")
    report_lines.append("=" * 100)
    report_lines.append("")
    
    # 收集所有重要文件
    important_files = []
    
    print("正在扫描文件...")
    for root, dirs, files in os.walk(root_path):
        # 过滤排除目录
        dirs[:] = [d for d in dirs if not should_exclude(Path(root)/d, options)]
        
        for file in files:
            filepath = Path(root)/file
            
            if should_exclude(filepath, options):
                continue
            
            if is_important_file(filepath):
                important_files.append(filepath)
    
    print(f"找到 {len(important_files)} 个重要文件")
    
    # 文件树结构
    if options['include_tree'] and important_files:
        report_lines.append("-" * 100)
        report_lines.append("🌳 文件树结构")
        report_lines.append("-" * 100)
        tree_lines = generate_tree_structure(root_path, options)
        report_lines.extend(tree_lines)
        report_lines.append("")
        report_lines.append("-" * 100)
        report_lines.append("")
    
    # 按目录分组显示所有文件内容
    if important_files:
        # 按目录分组
        files_by_dir = {}
        for filepath in important_files:
            parent = filepath.parent
            if parent not in files_by_dir:
                files_by_dir[parent] = []
            files_by_dir[parent].append(filepath)
        
        # 按目录名排序
        for dir_path in sorted(files_by_dir.keys()):
            rel_dir = dir_path.relative_to(root_path)
            if str(rel_dir) == '.':
                report_lines.append(f"\n{'=' * 100}")
                report_lines.append(f"📁 根目录")
                report_lines.append(f"{'=' * 100}")
            else:
                report_lines.append(f"\n{'=' * 100}")
                report_lines.append(f"📁 {rel_dir}/")
                report_lines.append(f"{'=' * 100}")
            
            # 显示该目录下的所有文件
            for filepath in sorted(files_by_dir[dir_path]):
                report_lines.append(f"\n{'-' * 100}")
                report_lines.append(f"📄 {filepath.name}")
                report_lines.append(f"{'-' * 100}")
                
                # 读取完整的文件内容
                content = read_file_content(filepath)
                if content is not None:
                    if content == "":  # 空文件
                        report_lines.append("(空文件)")
                    else:
                        report_lines.append(content)
                else:
                    report_lines.append("(无法读取文件内容 - 可能是二进制文件)")
                
                report_lines.append("")  # 空行分隔
    
    # 结尾
    report_lines.append("\n" + "=" * 100)
    report_lines.append(f"提取完成 - 共处理 {len(important_files)} 个文件")
    report_lines.append("=" * 100)
    
    return '\n'.join(report_lines)

def main():
    """主函数"""
    # 检查是否有tkinter
    try:
        import tkinter
    except ImportError:
        print("错误: 需要tkinter支持")
        print("在Ubuntu/Debian上安装: sudo apt-get install python3-tk")
        print("在CentOS/RHEL上安装: sudo yum install python3-tkinter")
        sys.exit(1)
    
    # 运行GUI
    app = ReportGeneratorGUI()
    app.run()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n❌ 用户中断")
        sys.exit(0)
    except Exception as e:
        print(f"\n❌ 发生错误: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
