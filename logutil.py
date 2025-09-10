import logging, os
from typing import Optional

def setup_logger(log_path: str, name: str = "mavviz", console_level: int = logging.INFO, file_level: int = logging.DEBUG) -> logging.Logger:
    logger = logging.getLogger(name)
    logger.setLevel(logging.DEBUG)
    logger.handlers.clear()
    fmt = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
    ch = logging.StreamHandler(); ch.setLevel(console_level); ch.setFormatter(fmt)
    logger.addHandler(ch)
    try:
        if log_path:
            d = log_path.rsplit('/',1)[0]
            if d and d != log_path:
                os.makedirs(d, exist_ok=True)
            fh = logging.FileHandler(log_path, mode='w')
            fh.setLevel(file_level); fh.setFormatter(fmt); logger.addHandler(fh)
    except Exception as e:
        logger.warning(f"Failed to create log file: {e}")
    return logger
