# Python 踩坑记录

> 仅供本人查漏补缺，有任何问题可以留言或邮箱交流
> [Contact me](mailto:Ljian1997@gmail.com)

## & 和 and 的区别

例如以下例子：

```python
val.val != 1 &/and value == val.val
```

`&` : 先计算 `val.val != 1 & value` 再判断是否与 `val.val` 相等

`and` : 先计算 `val.val != 1` 和 `value == val.val`, 再判断 `and`

## * 和 ** 的使用

### 形参时

- `*args` ：用于收集位置参数为一个元组。
- `**kwargs` ：用于收集关键字参数为一个字典。

例如：

```python
def my_function(a, b, *args, **kwargs):
    print("a =", a)
    print("b =", b)
    print("args =", args)
    print("kwargs =", kwargs)

my_function(1, 2, 3, 4, 5, name="Alice", age=25)
```

输出：

```python
a = 1
b = 2
args = (3, 4, 5)
kwargs = {'name': 'Alice', 'age': 25}
```

### 实参时

带 `*` 或 `**` 表示 `list` 或字典内的元素，不带则表示 `list` 或字典

```python
>>> def fun(data1, data2, data3):
...     print("data1: ", data1)
...     print("data2: ", data2)
...     print("data3: ", data3)
... 
>>> args = ("one", 2, 3)
>>> fun(*args)
data1:  one
data2:  2
data3:  3
>>> kwargs = {"data3": "one", "data2": 2, "data1": 3}
>>> fun(**kwargs)
data1:  3
data2:  2
data3:  one

>>> a, b, *c = 0, 1, 2, 3  # 序列解包没有 **
>>> a  
0  
>>> b  
1  
>>> c  
[2, 3]
```

## @ 的使用

> @ 用于定义装饰器。装饰器是一种特殊的函数，可以在不修改被装饰函数的源代码的情况下，对函数进行扩展或修改。
>
> > 装饰器的主要作用是在函数定义之前或之后添加额外的功能，例如日志记录、参数验证、性能分析等。装饰器可以接受一个函数作为参数，并返回一个新的函数，这个新函数将替换原来的函数。

例子：

```python
def timing_decorator(func):
    def wrapper(*args, **kwargs):
        import time
        start_time = time.time()
        result = func(*args, **kwargs)
        end_time = time.time()
        print(f"函数 {func.__name__} 执行耗时: {end_time - start_time} 秒")
        return result
    return wrapper

@timing_decorator
def long_running_function(n):
    total = 0
    for i in range(n):
        total += i
    return total

print(long_running_function(100000))
```
## logging 的使用

```Python
import logging
import argparse
import time

def create_logger(logger_file_path):

    if not os.path.exists(logger_file_path):
        os.makedirs(logger_file_path)
    log_name = '{}.log'.format(time.strftime('%Y-%m-%d-%H-%M'))
    final_log_file = os.path.join(logger_file_path, log_name)

    logger = logging.getLogger()  # 设定日志对象
    logger.setLevel(logging.INFO)  # 设定日志等级

    file_handler = logging.FileHandler(final_log_file)  # 文件输出
    console_handler = logging.StreamHandler()  # 控制台输出

    # 输出格式
    formatter = logging.Formatter(
        "%(asctime)s %(levelname)s: %(message)s "
    )

    file_handler.setFormatter(formatter)  # 设置文件输出格式
    console_handler.setFormatter(formatter)  # 设施控制台输出格式
    logger.addHandler(file_handler)
    logger.addHandler(console_handler)

    return logger

# Usage
if __name__ == '__main__':
     parser = argparse.ArgumentParser(description='configTemplates')
     parser.add_argument('-log_path', default='./results', type=str, help='log file path to save result')

     args = parser.parse_args()

     logger = create_logger(args.log_path)
     logger.info('------Begin Training Model------')
     cIOU = metric.evaluate()[0][1]
     cIOU *= 100.0
     pixAcc = 1.0 * total_correct / (np.spacing(1) + total_label)
     #计算出当前结果后调用info函数进行写入日志文件
     logger.info('cIOU:  {}, PA:  {}'.format(cIOU, pixAcc))

```

或者下列代码

```Python
import logging
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
```
