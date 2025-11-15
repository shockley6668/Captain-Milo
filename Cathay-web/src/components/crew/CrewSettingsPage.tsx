import { Settings, Bell, User, Shield, Globe } from 'lucide-react';
import { Card } from '../ui/card';
import { Switch } from '../ui/switch';
import { Button } from '../ui/button';
import { Badge } from '../ui/badge';

export function CrewSettingsPage() {
  return (
    <>
      {/* Header */}
      <div className="mb-6">
        <div className="bg-[var(--cathay-bg-card)] rounded-xl p-6 shadow-lg border border-[var(--cathay-border)]">
          <h1 className="text-[var(--cathay-text-primary)] mb-2 text-xl font-semibold">系统设置</h1>
          <p className="text-[var(--cathay-text-secondary)]">配置监控系统和个人偏好</p>
        </div>
      </div>

      {/* Profile Settings */}
      <Card className="bg-[var(--cathay-bg-card)] border border-[var(--cathay-border)] shadow-lg p-6 mb-6">
        <div className="flex items-center mb-4">
          <User className="w-5 h-5 text-[var(--cathay-jade-dark)] mr-2" />
          <h3 className="text-[var(--cathay-text-primary)] font-semibold text-lg">个人信息</h3>
        </div>
        <div className="flex items-start gap-6">
          <div className="w-20 h-20 rounded-full bg-[var(--cathay-bg-subtle)] flex items-center justify-center">
            <User className="w-10 h-10 text-[var(--cathay-jade-dark)]" />
          </div>
          <div className="flex-1">
            <div className="flex items-center gap-3 mb-2">
              <h3 className="text-[var(--cathay-text-primary)] text-xl font-semibold">张丽</h3>
              <Badge className="bg-[var(--cathay-jade-dark)] text-white">高级乘务员</Badge>
            </div>
            <div className="space-y-2 text-[var(--cathay-text-secondary)]">
              <p>员工编号: CX-2024-1234</p>
              <p>航班: CX889 · 香港 → 洛杉矶</p>
              <p>服务经验: 8年</p>
            </div>
            <Button 
              variant="outline" 
              className="mt-4 border-[var(--cathay-border)] text-[var(--cathay-text-primary)]"
            >
              编辑个人信息
            </Button>
          </div>
        </div>
      </Card>

      {/* Alert Settings */}
      <Card className="bg-[var(--cathay-bg-card)] border border-[var(--cathay-border)] shadow-lg p-6 mb-6">
        <div className="flex items-center mb-4">
          <Bell className="w-5 h-5 text-[var(--cathay-jade-dark)] mr-2" />
          <h3 className="text-[var(--cathay-text-primary)] font-semibold text-lg">提醒设置</h3>
        </div>
        <div className="space-y-4">
          <div className="flex items-center justify-between p-4 bg-[var(--cathay-bg-subtle)] rounded-lg border border-[var(--cathay-border)]">
            <div>
              <div className="text-[var(--cathay-text-primary)] font-medium">高优先级提醒</div>
              <div className="text-[var(--cathay-text-secondary)] text-sm">立即通知高优先级事件</div>
            </div>
            <Switch defaultChecked />
          </div>
          <div className="flex items-center justify-between p-4 bg-[var(--cathay-bg-subtle)] rounded-lg border border-[var(--cathay-border)]">
            <div>
              <div className="text-[var(--cathay-text-primary)] font-medium">情绪波动提醒</div>
              <div className="text-[var(--cathay-text-secondary)] text-sm">儿童情绪评分低于70时提醒</div>
            </div>
            <Switch defaultChecked />
          </div>
          <div className="flex items-center justify-between p-4 bg-[var(--cathay-bg-subtle)] rounded-lg border border-[var(--cathay-border)]">
            <div>
              <div className="text-[var(--cathay-text-primary)] font-medium">生命体征异常</div>
              <div className="text-[var(--cathay-text-secondary)] text-sm">体温或心率异常时立即通知</div>
            </div>
            <Switch defaultChecked />
          </div>
          <div className="flex items-center justify-between p-4 bg-[var(--cathay-bg-subtle)] rounded-lg border border-[var(--cathay-border)]">
            <div>
              <div className="text-[var(--cathay-text-primary)] font-medium">服务请求通知</div>
              <div className="text-[var(--cathay-text-secondary)] text-sm">儿童服务请求实时推送</div>
            </div>
            <Switch defaultChecked />
          </div>
          <div className="flex items-center justify-between p-4 bg-[var(--cathay-bg-subtle)] rounded-lg border border-[var(--cathay-border)]">
            <div>
              <div className="text-[var(--cathay-text-primary)] font-medium">声音提醒</div>
              <div className="text-[var(--cathay-text-secondary)] text-sm">紧急情况时播放提醒音</div>
            </div>
            <Switch defaultChecked />
          </div>
        </div>
      </Card>

      {/* Monitoring Settings */}
      <Card className="bg-[var(--cathay-bg-card)] border border-[var(--cathay-border)] shadow-lg p-6 mb-6">
        <div className="flex items-center mb-4">
          <Shield className="w-5 h-5 text-[var(--cathay-jade-dark)] mr-2" />
          <h3 className="text-[var(--cathay-text-primary)] font-semibold text-lg">监控设置</h3>
        </div>
        <div className="space-y-4">
          <div className="flex items-center justify-between p-4 bg-[var(--cathay-bg-subtle)] rounded-lg border border-[var(--cathay-border)]">
            <div>
              <div className="text-[var(--cathay-text-primary)] font-medium">实时数据刷新</div>
              <div className="text-[var(--cathay-text-secondary)] text-sm">每30秒自动刷新监控数据</div>
            </div>
            <Switch defaultChecked />
          </div>
          <div className="flex items-center justify-between p-4 bg-[var(--cathay-bg-subtle)] rounded-lg border border-[var(--cathay-border)]">
            <div>
              <div className="text-[var(--cathay-text-primary)] font-medium">详细日志记录</div>
              <div className="text-[var(--cathay-text-secondary)] text-sm">记录所有儿童活动和事件</div>
            </div>
            <Switch defaultChecked />
          </div>
          <div className="flex items-center justify-between p-4 bg-[var(--cathay-bg-subtle)] rounded-lg border border-[var(--cathay-border)]">
            <div>
              <div className="text-[var(--cathay-text-primary)] font-medium">自动报告生成</div>
              <div className="text-[var(--cathay-text-secondary)] text-sm">航班结束时自动生成服务报告</div>
            </div>
            <Switch defaultChecked />
          </div>
        </div>
      </Card>

      {/* Display Settings */}
      <Card className="bg-[var(--cathay-bg-card)] border border-[var(--cathay-border)] shadow-lg p-6 mb-6">
        <div className="flex items-center mb-4">
          <Globe className="w-5 h-5 text-[var(--cathay-jade-dark)] mr-2" />
          <h3 className="text-[var(--cathay-text-primary)] font-semibold text-lg">显示设置</h3>
        </div>
        <div className="space-y-4">
          <div className="flex items-center justify-between p-4 bg-[var(--cathay-bg-subtle)] rounded-lg border border-[var(--cathay-border)]">
            <div>
              <div className="text-[var(--cathay-text-primary)] font-medium">紧凑模式</div>
              <div className="text-[var(--cathay-text-secondary)] text-sm">在更小的空间内显示更多信息</div>
            </div>
            <Switch />
          </div>
          <div className="flex items-center justify-between p-4 bg-[var(--cathay-bg-subtle)] rounded-lg border border-[var(--cathay-border)]">
            <div className="flex items-center gap-2">
              <div className="text-[var(--cathay-text-primary)] font-medium">界面语言</div>
            </div>
            <select className="bg-white border border-[var(--cathay-border)] text-[var(--cathay-text-primary)] rounded-lg px-3 py-2">
              <option>简体中文</option>
              <option>English</option>
              <option>繁體中文</option>
            </select>
          </div>
        </div>
      </Card>

      {/* Action Buttons */}
      <Card className="bg-[var(--cathay-bg-card)] border border-[var(--cathay-border)] shadow-lg p-6">
        <h3 className="text-[var(--cathay-text-primary)] font-semibold text-lg mb-4">系统操作</h3>
        <div className="space-y-3">
          <Button className="w-full bg-[var(--cathay-jade-dark)] text-white hover:bg-[var(--cathay-jade)]">
            保存所有更改
          </Button>
          <Button 
            variant="outline" 
            className="w-full border-[var(--cathay-border)] text-[var(--cathay-text-primary)]"
          >
            重置为默认设置
          </Button>
          <Button 
            variant="outline" 
            className="w-full border-red-500 text-red-600 hover:bg-red-50"
          >
            退出登录
          </Button>
        </div>
      </Card>
    </>
  );
}
