# Cathay Web Service Deployment Guide

## Service Information

- **Application Name**: Cathay Web Application
- **Port**: 5000
- **Domain**: jarvis-pi.dynv6.net
- **Service Name**: cathay-web.service
- **Build Directory**: /home/jarvis-pi/cathay_web/build

## Service Management Commands

### Check Service Status
```bash
sudo systemctl status cathay-web.service
```

### Start Service
```bash
sudo systemctl start cathay-web.service
```

### Stop Service
```bash
sudo systemctl stop cathay-web.service
```

### Restart Service
```bash
sudo systemctl restart cathay-web.service
```

### View Service Logs
```bash
sudo journalctl -u cathay-web.service -f
```

### Disable Auto-Start on Boot
```bash
sudo systemctl disable cathay-web.service
```

### Enable Auto-Start on Boot
```bash
sudo systemctl enable cathay-web.service
```

## Update the Application

After updating the code, run the following commands to rebuild and redeploy:

```bash
cd /home/jarvis-pi/cathay_web
npm run build
sudo systemctl restart cathay-web.service
```

## Access the Application

- **Local Access**: http://localhost:5000
- **LAN Access**: http://jarvis-pi.local:5000
- **Public Access**: http://jarvis-pi.dynv6.net:5000

## Service Configuration File

Service configuration file location: `/etc/systemd/system/cathay-web.service`

To modify the configuration:
```bash
sudo nano /etc/systemd/system/cathay-web.service
sudo systemctl daemon-reload
sudo systemctl restart cathay-web.service
```

## Troubleshooting

1. Check whether the port is in use:
```bash
sudo ss -tlnp | grep :5000
```

2. Inspect service logs:
```bash
sudo journalctl -u cathay-web.service --no-pager | tail -50
```

3. Manually test the static serve:
```bash
cd /home/jarvis-pi/cathay_web
serve -s build -l 5000
```

## Notes

- The service starts automatically with the system
- If the service crashes, it restarts automatically (10-second interval)
- The application runs in production mode
- The service runs under the `jarvis-pi` user
