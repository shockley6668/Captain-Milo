# Cathay Web 服务部署说明

## 服务信息

- **应用名称**: Cathay Web Application
- **端口**: 5000
- **域名**: jarvis-pi.dynv6.net
- **服务名**: cathay-web.service
- **构建目录**: /home/jarvis-pi/cathay_web/build

## 服务管理命令

### 查看服务状态
```bash
sudo systemctl status cathay-web.service
```

### 启动服务
```bash
sudo systemctl start cathay-web.service
```

### 停止服务
```bash
sudo systemctl stop cathay-web.service
```

### 重启服务
```bash
sudo systemctl restart cathay-web.service
```

### 查看服务日志
```bash
sudo journalctl -u cathay-web.service -f
```

### 禁用开机自启动
```bash
sudo systemctl disable cathay-web.service
```

### 启用开机自启动
```bash
sudo systemctl enable cathay-web.service
```

## 更新应用

当你更新代码后，运行以下命令重新构建和部署：

```bash
cd /home/jarvis-pi/cathay_web
npm run build
sudo systemctl restart cathay-web.service
```

## 访问应用

- **本地访问**: http://localhost:5000
- **局域网访问**: http://jarvis-pi.local:5000
- **公网访问**: http://jarvis-pi.dynv6.net:5000

## 服务配置文件

服务配置文件位置: `/etc/systemd/system/cathay-web.service`

如需修改配置:
```bash
sudo nano /etc/systemd/system/cathay-web.service
sudo systemctl daemon-reload
sudo systemctl restart cathay-web.service
```

## 故障排查

1. 检查端口是否被占用:
```bash
sudo ss -tlnp | grep :5000
```

2. 检查服务日志:
```bash
sudo journalctl -u cathay-web.service --no-pager | tail -50
```

3. 手动测试 serve:
```bash
cd /home/jarvis-pi/cathay_web
serve -s build -l 5000
```

## 注意事项

- 服务会在系统启动时自动运行
- 如果服务崩溃，会自动重启（重启间隔10秒）
- 应用运行在生产模式下
- 使用用户 `jarvis-pi` 运行服务
