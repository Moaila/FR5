import logging
from colorama import Fore, Style, init

# 初始化 colorama（Windows 上特别需要，其他平台也兼容）
init(autoreset=True)

class CustomFormatter(logging.Formatter):
    """自定义日志格式，支持颜色显示"""
    
    # 日志级别对应的颜色
    format_dict = {
        logging.INFO: Fore.GREEN + "%(asctime)s - %(levelname)s - %(message)s" + Style.RESET_ALL,
        logging.ERROR: Fore.RED + "%(asctime)s - %(levelname)s - %(message)s" + Style.RESET_ALL
    }

    def format(self, record):
        log_fmt = self.format_dict.get(record.levelno, "%(asctime)s - %(levelname)s - %(message)s")
        formatter = logging.Formatter(log_fmt, "%Y-%m-%d %H:%M:%S")
        return formatter.format(record)

def setup_logger(name, log_file, level=logging.INFO):
    """设置日志配置，支持输出到文件和控制台，控制台带颜色"""

    # 创建 logger
    logger = logging.getLogger(name)
    logger.setLevel(level)
    
    # 创建文件 handler，用于输出日志到文件
    file_handler = logging.FileHandler(log_file)
    file_handler.setLevel(level)
    file_formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s', "%Y-%m-%d %H:%M:%S")
    file_handler.setFormatter(file_formatter)
    
    # 创建控制台 handler，用于控制台输出日志（带颜色）
    console_handler = logging.StreamHandler()
    console_handler.setLevel(level)
    console_handler.setFormatter(CustomFormatter())
    
    # 将 handlers 添加到 logger
    if not logger.hasHandlers():
        logger.addHandler(file_handler)
        logger.addHandler(console_handler)
    
    return logger
