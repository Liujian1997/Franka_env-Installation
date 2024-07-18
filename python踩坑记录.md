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