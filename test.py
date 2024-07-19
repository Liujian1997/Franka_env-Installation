import logging
import argparse
import datetime
import os

import logging
def logger_init(log_file_name='monitor',
                log_level=logging.DEBUG,
                log_dir='./logs/',
                only_file=False):
     # 指定路径
     if not os.path.exists(log_dir):
          os.makedirs(log_dir)

     log_path = os.path.join(log_dir, log_file_name + '_' + str(datetime.datetime.now())[:10] + '.txt')
     formatter = '[%(asctime)s] - %(levelname)s: %(message)s'
     if only_file:
          logging.basicConfig(filename=log_path,
                              level=log_level,
                              format=formatter,
                              datefmt='%Y-%d-%m %H:%M:%S')
     else:
          logging.basicConfig(level=log_level,
                              format=formatter,
                              datefmt='%Y-%d-%m %H:%M:%S',
                              handlers=[logging.FileHandler(log_path),
                                        logging.StreamHandler()]
                              )

# Usage
if __name__ == '__main__':
     logger_init()

     logging.debug('This is a debug message')

     logging.info('This is an info message')

     logging.warning('This is a warning message')

     logging.error('This is an error message')

     logging.critical('This is a critical message')