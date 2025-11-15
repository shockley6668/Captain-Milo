import { Settings, Bell, Lock, Globe, Moon, Shield } from 'lucide-react';
import { Card } from '../ui/card';
import { Switch } from '../ui/switch';
import { Button } from '../ui/button';

export function SettingsPage() {
  return (
    <>
      {/* Header */}
      <div className="mb-6">
        <div className="bg-[var(--cathay-bg-card)] rounded-xl p-6 shadow-lg border border-[var(--cathay-border)]">
          <h1 className="text-[var(--cathay-text-primary)] mb-2 text-xl font-semibold">系统设置</h1>
          <p className="text-[var(--cathay-text-secondary)]">管理您的偏好设置和账户信息</p>
        </div>
      </div>

      {/* Notification Settings */}
      <Card className="bg-[var(--cathay-bg-card)] border border-[var(--cathay-border)] shadow-lg p-6 mb-6">
        <div className="flex items-center mb-4">
          <Bell className="w-5 h-5 text-[var(--cathay-jade-dark)] mr-2" />
          <h3 className="text-[var(--cathay-text-primary)] font-semibold text-lg">通知设置</h3>
        </div>
        <div className="space-y-4">
          <div className="flex items-center justify-between p-4 bg-[var(--cathay-bg-subtle)] rounded-lg border border-[var(--cathay-border)]">
            <div>
              <div className="text-[var(--cathay-text-primary)] font-medium">推送通知</div>
              <div className="text-[var(--cathay-text-secondary)] text-sm">接收实时旅行动态推送</div>
            </div>
            <Switch defaultChecked />
          </div>
          <div className="flex items-center justify-between p-4 bg-[var(--cathay-bg-subtle)] rounded-lg border border-[var(--cathay-border)]">
            <div>
              <div className="text-[var(--cathay-text-primary)] font-medium">邮件通知</div>
              <div className="text-[var(--cathay-text-secondary)] text-sm">通过邮件接收重要更新</div>
            </div>
            <Switch defaultChecked />
          </div>
          <div className="flex items-center justify-between p-4 bg-[var(--cathay-bg-subtle)] rounded-lg border border-[var(--cathay-border)]">
            <div>
              <div className="text-[var(--cathay-text-primary)] font-medium">短信提醒</div>
              <div className="text-[var(--cathay-text-secondary)] text-sm">重要事项短信提醒</div>
            </div>
            <Switch />
          </div>
          <div className="flex items-center justify-between p-4 bg-[var(--cathay-bg-subtle)] rounded-lg border border-[var(--cathay-border)]">
            <div>
              <div className="text-[var(--cathay-text-primary)] font-medium">情绪波动提醒</div>
              <div className="text-[var(--cathay-text-secondary)] text-sm">孩子情绪变化时立即通知</div>
            </div>
            <Switch defaultChecked />
          </div>
        </div>
      </Card>

      {/* Privacy & Security */}
      <Card className="bg-[var(--cathay-bg-card)] border border-[var(--cathay-border)] shadow-lg p-6 mb-6">
        <div className="flex items-center mb-4">
          <Shield className="w-5 h-5 text-[var(--cathay-jade-dark)] mr-2" />
          <h3 className="text-[var(--cathay-text-primary)] font-semibold text-lg">隐私与安全</h3>
        </div>
        <div className="space-y-4">
          <div className="flex items-center justify-between p-4 bg-[var(--cathay-bg-subtle)] rounded-lg border border-[var(--cathay-border)]">
            <div>
              <div className="text-[var(--cathay-text-primary)] font-medium">生物识别登录</div>
              <div className="text-[var(--cathay-text-secondary)] text-sm">使用指纹或面部识别</div>
            </div>
            <Switch defaultChecked />
          </div>
          <div className="flex items-center justify-between p-4 bg-[var(--cathay-bg-subtle)] rounded-lg border border-[var(--cathay-border)]">
            <div>
              <div className="text-[var(--cathay-text-primary)] font-medium">双因素认证</div>
              <div className="text-[var(--cathay-text-secondary)] text-sm">增强账户安全性</div>
            </div>
            <Switch defaultChecked />
          </div>
          <Button className="w-full bg-[var(--cathay-bg-subtle)] text-[var(--cathay-text-primary)] hover:bg-[var(--cathay-border)] border border-[var(--cathay-border)]">
            <Lock className="w-4 h-4 mr-2" />
            修改密码
          </Button>
        </div>
      </Card>

      {/* Display Settings */}
      <Card className="bg-[var(--cathay-bg-card)] border border-[var(--cathay-border)] shadow-lg p-6 mb-6">
        <div className="flex items-center mb-4">
          <Moon className="w-5 h-5 text-[var(--cathay-jade-dark)] mr-2" />
          <h3 className="text-[var(--cathay-text-primary)] font-semibold text-lg">显示设置</h3>
        </div>
        <div className="space-y-4">
          <div className="flex items-center justify-between p-4 bg-[var(--cathay-bg-subtle)] rounded-lg border border-[var(--cathay-border)]">
            <div>
              <div className="text-[var(--cathay-text-primary)] font-medium">深色模式</div>
              <div className="text-[var(--cathay-text-secondary)] text-sm">切换至深色主题</div>
            </div>
            <Switch />
          </div>
          <div className="flex items-center justify-between p-4 bg-[var(--cathay-bg-subtle)] rounded-lg border border-[var(--cathay-border)]">
            <div className="flex items-center gap-2">
              <Globe className="w-4 h-4 text-[var(--cathay-jade-dark)]" />
              <div className="text-[var(--cathay-text-primary)] font-medium">语言</div>
            </div>
            <select className="bg-white border border-[var(--cathay-border)] text-[var(--cathay-text-primary)] rounded-lg px-3 py-2">
              <option>简体中文</option>
              <option>English</option>
              <option>繁體中文</option>
            </select>
          </div>
        </div>
      </Card>

      {/* Account Actions */}
      <Card className="bg-[var(--cathay-bg-card)] border border-[var(--cathay-border)] shadow-lg p-6">
        <h3 className="text-[var(--cathay-text-primary)] font-semibold text-lg mb-4">账户操作</h3>
        <div className="space-y-3">
          <Button className="w-full bg-[var(--cathay-jade-dark)] text-white hover:bg-[var(--cathay-jade)]">
            保存更改
          </Button>
          <Button variant="outline" className="w-full border-[var(--cathay-border)] text-[var(--cathay-text-primary)]">
            退出登录
          </Button>
        </div>
      </Card>
    </>
  );
}
