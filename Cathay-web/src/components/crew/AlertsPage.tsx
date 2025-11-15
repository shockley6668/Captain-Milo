import { AlertTriangle, Info, AlertCircle, CheckCircle, Clock } from 'lucide-react';
import { Card } from '../ui/card';
import { Badge } from '../ui/badge';
import { Button } from '../ui/button';

export function AlertsPage() {
  const alerts = [
    {
      id: 1,
      priority: 'high',
      type: 'emotion',
      childName: '小华',
      seat: '18C',
      title: '情绪波动检测',
      message: '儿童情绪评分下降至65，可能感到想家或不适。',
      time: '5分钟前',
      status: 'pending',
      actions: ['查看详情', '前往座位', '联系家长']
    },
    {
      id: 2,
      priority: 'medium',
      type: 'request',
      childName: '小明',
      seat: '12A',
      title: '服务请求',
      message: '儿童请求饮料和小食。',
      time: '10分钟前',
      status: 'pending',
      actions: ['确认处理', '分配任务']
    },
    {
      id: 3,
      priority: 'low',
      type: 'info',
      childName: '小丽',
      seat: '20D',
      title: '活动提醒',
      message: '儿童已午睡超过2小时，建议适时唤醒。',
      time: '15分钟前',
      status: 'pending',
      actions: ['标记已读', '设置提醒']
    },
    {
      id: 4,
      priority: 'high',
      type: 'vital',
      childName: '小红',
      seat: '15B',
      title: '体温略高',
      message: '体温37.2°C，略高于正常值，建议关注。',
      time: '20分钟前',
      status: 'resolved',
      actions: ['查看处理记录']
    },
    {
      id: 5,
      priority: 'medium',
      type: 'emotion',
      childName: '小强',
      seat: '22A',
      title: '过度兴奋',
      message: '儿童情绪过于激动，建议进行安抚活动。',
      time: '30分钟前',
      status: 'resolved',
      actions: ['查看处理记录']
    }
  ];

  const getPriorityBadge = (priority: string) => {
    switch (priority) {
      case 'high':
        return <Badge className="bg-red-500 text-white">高优先级</Badge>;
      case 'medium':
        return <Badge className="bg-amber-500 text-white">中优先级</Badge>;
      case 'low':
        return <Badge className="bg-blue-500 text-white">低优先级</Badge>;
      default:
        return <Badge className="bg-gray-500 text-white">未知</Badge>;
    }
  };

  const getAlertIcon = (type: string) => {
    switch (type) {
      case 'emotion':
        return <AlertCircle className="w-6 h-6 text-amber-500" />;
      case 'vital':
        return <AlertTriangle className="w-6 h-6 text-red-500" />;
      case 'request':
        return <Info className="w-6 h-6 text-blue-500" />;
      default:
        return <Info className="w-6 h-6 text-[var(--cathay-jade-dark)]" />;
    }
  };

  const getStatusBadge = (status: string) => {
    if (status === 'resolved') {
      return <Badge variant="outline" className="border-green-500 text-green-600">已处理</Badge>;
    }
    return <Badge variant="outline" className="border-[var(--cathay-jade-dark)] text-[var(--cathay-jade-dark)]">待处理</Badge>;
  };

  const pendingAlerts = alerts.filter(a => a.status === 'pending');
  const resolvedAlerts = alerts.filter(a => a.status === 'resolved');

  return (
    <>
      {/* Header */}
      <div className="mb-6">
        <div className="bg-[var(--cathay-bg-card)] rounded-xl p-6 shadow-lg border border-[var(--cathay-border)]">
          <div className="flex items-center justify-between">
            <div>
              <h1 className="text-[var(--cathay-text-primary)] mb-2 text-xl font-semibold">提醒中心</h1>
              <p className="text-[var(--cathay-text-secondary)]">
                {pendingAlerts.length} 条待处理提醒 · {resolvedAlerts.length} 条已处理
              </p>
            </div>
            <Button className="bg-[var(--cathay-jade-dark)] text-white hover:bg-[var(--cathay-jade)]">
              全部标记已读
            </Button>
          </div>
        </div>
      </div>

      {/* Summary Cards */}
      <div className="grid md:grid-cols-3 gap-6 mb-6">
        <Card className="bg-red-50 border border-red-200 shadow-md p-4">
          <div className="flex items-center justify-between">
            <div>
              <div className="text-red-600 text-2xl font-bold">
                {alerts.filter(a => a.priority === 'high' && a.status === 'pending').length}
              </div>
              <div className="text-red-700 text-sm">高优先级待处理</div>
            </div>
            <AlertTriangle className="w-8 h-8 text-red-500" />
          </div>
        </Card>
        <Card className="bg-amber-50 border border-amber-200 shadow-md p-4">
          <div className="flex items-center justify-between">
            <div>
              <div className="text-amber-600 text-2xl font-bold">
                {alerts.filter(a => a.priority === 'medium' && a.status === 'pending').length}
              </div>
              <div className="text-amber-700 text-sm">中优先级待处理</div>
            </div>
            <AlertCircle className="w-8 h-8 text-amber-500" />
          </div>
        </Card>
        <Card className="bg-green-50 border border-green-200 shadow-md p-4">
          <div className="flex items-center justify-between">
            <div>
              <div className="text-green-600 text-2xl font-bold">{resolvedAlerts.length}</div>
              <div className="text-green-700 text-sm">今日已处理</div>
            </div>
            <CheckCircle className="w-8 h-8 text-green-500" />
          </div>
        </Card>
      </div>

      {/* Pending Alerts */}
      {pendingAlerts.length > 0 && (
        <div className="mb-6">
          <h3 className="text-[var(--cathay-text-primary)] font-semibold text-lg mb-4">待处理提醒</h3>
          <div className="space-y-4">
            {pendingAlerts.map((alert) => (
              <Card 
                key={alert.id}
                className="bg-[var(--cathay-bg-card)] border border-[var(--cathay-border)] shadow-lg p-6"
              >
                <div className="flex gap-4">
                  <div className="flex-shrink-0">
                    {getAlertIcon(alert.type)}
                  </div>
                  <div className="flex-1">
                    <div className="flex items-start justify-between mb-3">
                      <div>
                        <div className="flex items-center gap-2 mb-2">
                          <h3 className="text-[var(--cathay-text-primary)] font-semibold">{alert.title}</h3>
                          {getPriorityBadge(alert.priority)}
                          {getStatusBadge(alert.status)}
                        </div>
                        <p className="text-[var(--cathay-text-secondary)] text-sm">
                          {alert.childName} · 座位 {alert.seat}
                        </p>
                      </div>
                      <div className="flex items-center gap-1 text-[var(--cathay-text-secondary)] text-sm">
                        <Clock className="w-4 h-4" />
                        {alert.time}
                      </div>
                    </div>
                    <p className="text-[var(--cathay-text-secondary)] mb-4">{alert.message}</p>
                    <div className="flex gap-2">
                      {alert.actions.map((action, index) => (
                        <Button 
                          key={index}
                          size="sm"
                          variant={index === 0 ? 'default' : 'outline'}
                          className={
                            index === 0 
                              ? 'bg-[var(--cathay-jade-dark)] text-white hover:bg-[var(--cathay-jade)]'
                              : 'border-[var(--cathay-border)] text-[var(--cathay-text-primary)]'
                          }
                        >
                          {action}
                        </Button>
                      ))}
                    </div>
                  </div>
                </div>
              </Card>
            ))}
          </div>
        </div>
      )}

      {/* Resolved Alerts */}
      {resolvedAlerts.length > 0 && (
        <div>
          <h3 className="text-[var(--cathay-text-primary)] font-semibold text-lg mb-4">已处理提醒</h3>
          <div className="space-y-4">
            {resolvedAlerts.map((alert) => (
              <Card 
                key={alert.id}
                className="bg-[var(--cathay-bg-subtle)] border border-[var(--cathay-border)] shadow-md p-6 opacity-75"
              >
                <div className="flex gap-4">
                  <div className="flex-shrink-0">
                    <CheckCircle className="w-6 h-6 text-green-500" />
                  </div>
                  <div className="flex-1">
                    <div className="flex items-start justify-between mb-3">
                      <div>
                        <div className="flex items-center gap-2 mb-2">
                          <h3 className="text-[var(--cathay-text-primary)] font-semibold">{alert.title}</h3>
                          {getStatusBadge(alert.status)}
                        </div>
                        <p className="text-[var(--cathay-text-secondary)] text-sm">
                          {alert.childName} · 座位 {alert.seat}
                        </p>
                      </div>
                      <span className="text-[var(--cathay-text-secondary)] text-sm">{alert.time}</span>
                    </div>
                    <p className="text-[var(--cathay-text-secondary)] mb-4">{alert.message}</p>
                    <Button 
                      size="sm"
                      variant="outline"
                      className="border-[var(--cathay-border)] text-[var(--cathay-text-primary)]"
                    >
                      查看处理记录
                    </Button>
                  </div>
                </div>
              </Card>
            ))}
          </div>
        </div>
      )}
    </>
  );
}
