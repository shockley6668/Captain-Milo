import { Bell, Check, Info, AlertCircle } from 'lucide-react';
import { Card } from '../ui/card';
import { Badge } from '../ui/badge';
import { Button } from '../ui/button';

export function NotificationsPage() {
  const notifications = [
    {
      id: 1,
      type: 'success',
      title: '孩子已安全登机',
      message: '您的孩子已在乘务员陪同下安全登机，航班CX889准时起飞。',
      time: '10分钟前',
      read: false
    },
    {
      id: 2,
      type: 'info',
      title: '午餐时间',
      message: '孩子正在享用儿童餐，食欲良好。',
      time: '1小时前',
      read: false
    },
    {
      id: 3,
      type: 'info',
      title: '娱乐活动',
      message: '孩子正在观看动画片《狮子王》，情绪愉快。',
      time: '2小时前',
      read: true
    },
    {
      id: 4,
      type: 'success',
      title: '健康检查',
      message: '生命体征正常，体温36.5°C，心率78 BPM。',
      time: '3小时前',
      read: true
    },
    {
      id: 5,
      type: 'warning',
      title: '情绪波动提醒',
      message: '检测到孩子有些想家，乘务员已提供安抚，现在情绪稳定。',
      time: '4小时前',
      read: true
    },
    {
      id: 6,
      type: 'info',
      title: '航班状态更新',
      message: '航班飞行平稳，预计准时到达洛杉矶。',
      time: '5小时前',
      read: true
    }
  ];

  const getNotificationIcon = (type: string) => {
    switch (type) {
      case 'success':
        return <Check className="w-5 h-5 text-green-500" />;
      case 'warning':
        return <AlertCircle className="w-5 h-5 text-amber-500" />;
      default:
        return <Info className="w-5 h-5 text-[var(--cathay-jade-dark)]" />;
    }
  };

  const getNotificationBg = (type: string) => {
    switch (type) {
      case 'success':
        return 'bg-green-50 border-green-200';
      case 'warning':
        return 'bg-amber-50 border-amber-200';
      default:
        return 'bg-blue-50 border-blue-200';
    }
  };

  return (
    <>
      {/* Header */}
      <div className="mb-6">
        <div className="bg-[var(--cathay-bg-card)] rounded-xl p-6 shadow-lg border border-[var(--cathay-border)]">
          <div className="flex items-center justify-between">
            <div>
              <h1 className="text-[var(--cathay-text-primary)] mb-2 text-xl font-semibold">通知中心</h1>
              <p className="text-[var(--cathay-text-secondary)]">实时接收孩子的旅行动态</p>
            </div>
            <Button variant="outline" className="border-[var(--cathay-jade-dark)] text-[var(--cathay-jade-dark)]">
              全部标记已读
            </Button>
          </div>
        </div>
      </div>

      {/* Notifications List */}
      <div className="space-y-4 mb-6">
        {notifications.map((notification) => (
          <Card 
            key={notification.id}
            className={`${
              notification.read 
                ? 'bg-[var(--cathay-bg-card)]' 
                : 'bg-white'
            } border border-[var(--cathay-border)] shadow-lg p-6 transition-all hover:shadow-xl`}
          >
            <div className="flex gap-4">
              <div className={`flex-shrink-0 w-12 h-12 rounded-full ${getNotificationBg(notification.type)} flex items-center justify-center`}>
                {getNotificationIcon(notification.type)}
              </div>
              <div className="flex-1">
                <div className="flex items-start justify-between mb-2">
                  <div className="flex items-center gap-2">
                    <h3 className="text-[var(--cathay-text-primary)] font-semibold">{notification.title}</h3>
                    {!notification.read && (
                      <Badge className="bg-[var(--cathay-jade-dark)] text-white text-xs">新</Badge>
                    )}
                  </div>
                  <span className="text-[var(--cathay-text-secondary)] text-sm">{notification.time}</span>
                </div>
                <p className="text-[var(--cathay-text-secondary)]">{notification.message}</p>
              </div>
            </div>
          </Card>
        ))}
      </div>

      {/* Load More */}
      <div className="flex justify-center">
        <Button variant="outline" className="border-[var(--cathay-border)] text-[var(--cathay-text-primary)]">
          加载更多通知
        </Button>
      </div>
    </>
  );
}
