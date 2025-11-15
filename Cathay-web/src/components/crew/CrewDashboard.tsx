import { MapPin, Heart, Thermometer, Activity, AlertCircle, MessageSquare, Clock } from 'lucide-react';
import { Card } from '../ui/card';
import { Badge } from '../ui/badge';
import { Progress } from '../ui/progress';
import { EmotionChart } from '../EmotionChart';
import { VitalSignsChart } from '../VitalSignsChart';
import { CrewNavigation } from './CrewNavigation';
import { ChildrenListPage } from './ChildrenListPage';
import { AlertsPage } from './AlertsPage';
import { ReportsPage } from './ReportsPage';
import { CrewSettingsPage } from './CrewSettingsPage';
import { useState } from 'react';

interface CrewDashboardProps {
  childName: string;
  onLogout: () => void;
}

export function CrewDashboard({ childName, onLogout }: CrewDashboardProps) {
  const [activeTab, setActiveTab] = useState('dashboard');
  
  // 模拟数据
  const childInfo = {
    name: childName,
    age: 8,
    seat: '12A',
    flight: 'CX889',
    destination: '香港 → 洛杉矶',
    currentEmotion: '开心',
    emotionScore: 85,
    location: '座位区域',
    heartRate: 78,
    temperature: 36.5,
    lastUpdate: '2分钟前'
  };

  const alerts = [
    { id: 1, type: 'info', message: '儿童请求饮料', time: '5分钟前' },
    { id: 2, type: 'warning', message: '情绪波动检测', time: '15分钟前' }
  ];

  const recentActivities = [
    { id: 1, activity: '观看动画片', time: '10:30', duration: '30分钟' },
    { id: 2, activity: '与AI玩偶对话', time: '11:00', duration: '15分钟' },
    { id: 3, activity: '午餐时间', time: '11:30', duration: '20分钟' },
    { id: 4, activity: '阅读绘本', time: '12:00', duration: '进行中' }
  ];

  // Render content based on active tab
  const renderContent = () => {
    switch (activeTab) {
      case 'children':
        return <ChildrenListPage />;
      case 'alerts':
        return <AlertsPage />;
      case 'reports':
        return <ReportsPage />;
      case 'settings':
        return <CrewSettingsPage />;
      case 'dashboard':
      default:
        return renderDashboard();
    }
  };

  const renderDashboard = () => (
    <>
      {/* Header */}
      <div className="mb-6">
        <div className="flex items-center justify-between bg-[var(--cathay-bg-card)] rounded-xl p-6 shadow-lg border border-[var(--cathay-border)]">
          <div>
            <h1 className="text-[var(--cathay-text-primary)] mb-2 text-xl font-semibold">监控面板</h1>
            <p className="text-[var(--cathay-text-secondary)]">航班 {childInfo.flight} · {childInfo.destination}</p>
          </div>
          <div className="text-right">
            <div className="text-[var(--cathay-text-secondary)]">最后更新</div>
            <div className="text-[var(--cathay-text-primary)] flex items-center justify-end">
              <Clock className="w-4 h-4 mr-1" />
              {childInfo.lastUpdate}
            </div>
          </div>
        </div>
      </div>

        {/* Child Info Card */}
        <Card className="bg-[var(--cathay-bg-card)] border border-[var(--cathay-border)] shadow-lg p-6 mb-6">
          <div className="flex items-start justify-between mb-6">
            <div>
              <h2 className="text-[var(--cathay-text-primary)] mb-2 text-lg font-semibold">儿童信息</h2>
              <div className="space-y-2">
                <p className="text-[var(--cathay-text-secondary)]">姓名：{childInfo.name} · {childInfo.age}岁</p>
                <p className="text-[var(--cathay-text-secondary)]">座位：{childInfo.seat}</p>
              </div>
            </div>
            <Badge className="bg-[var(--cathay-jade-dark)] text-white">状态正常</Badge>
          </div>

          <div className="grid md:grid-cols-4 gap-4">
            <div className="bg-[var(--cathay-bg-subtle)] rounded-lg p-4 border border-[var(--cathay-border)]">
              <div className="flex items-center mb-2">
                <Activity className="w-5 h-5 text-[var(--cathay-jade-dark)] mr-2" />
                <span className="text-[var(--cathay-text-secondary)]">当前情绪</span>
              </div>
              <div className="text-[var(--cathay-text-primary)] font-medium">{childInfo.currentEmotion}</div>
              <Progress value={childInfo.emotionScore} className="mt-2" />
            </div>

            <div className="bg-[var(--cathay-bg-subtle)] rounded-lg p-4 border border-[var(--cathay-border)]">
              <div className="flex items-center mb-2">
                <MapPin className="w-5 h-5 text-[var(--cathay-jade-dark)] mr-2" />
                <span className="text-[var(--cathay-text-secondary)]">位置</span>
              </div>
              <div className="text-[var(--cathay-text-primary)] font-medium">{childInfo.location}</div>
            </div>

            <div className="bg-[var(--cathay-bg-subtle)] rounded-lg p-4 border border-[var(--cathay-border)]">
              <div className="flex items-center mb-2">
                <Heart className="w-5 h-5 text-[var(--cathay-jade-dark)] mr-2" />
                <span className="text-[var(--cathay-text-secondary)]">心率</span>
              </div>
              <div className="text-[var(--cathay-text-primary)] font-medium">{childInfo.heartRate} BPM</div>
            </div>

            <div className="bg-[var(--cathay-bg-subtle)] rounded-lg p-4 border border-[var(--cathay-border)]">
              <div className="flex items-center mb-2">
                <Thermometer className="w-5 h-5 text-[var(--cathay-jade-dark)] mr-2" />
                <span className="text-[var(--cathay-text-secondary)]">体温</span>
              </div>
              <div className="text-[var(--cathay-text-primary)] font-medium">{childInfo.temperature}°C</div>
            </div>
          </div>
        </Card>

        <div className="grid md:grid-cols-2 gap-6 mb-6">
          {/* Emotion Chart */}
          <Card className="bg-[var(--cathay-bg-card)] border border-[var(--cathay-border)] shadow-lg p-6">
            <h3 className="text-[var(--cathay-text-primary)] mb-4 font-semibold">情绪趋势分析</h3>
            <EmotionChart />
          </Card>

          {/* Vital Signs Chart */}
          <Card className="bg-[var(--cathay-bg-card)] border border-[var(--cathay-border)] shadow-lg p-6">
            <h3 className="text-[var(--cathay-text-primary)] mb-4 font-semibold">生命体征监测</h3>
            <VitalSignsChart />
          </Card>
        </div>

        <div className="grid md:grid-cols-2 gap-6">
          {/* Alerts */}
          <Card className="bg-[var(--cathay-bg-card)] border border-[var(--cathay-border)] shadow-lg p-6">
            <div className="flex items-center mb-4">
              <AlertCircle className="w-5 h-5 text-[var(--cathay-jade-dark)] mr-2" />
              <h3 className="text-[var(--cathay-text-primary)] font-semibold">提醒事项</h3>
            </div>
            <div className="space-y-3">
              {alerts.map(alert => (
                <div 
                  key={alert.id}
                  className={`p-4 rounded-lg border ${
                    alert.type === 'warning' 
                      ? 'bg-amber-50 border-amber-200' 
                      : 'bg-blue-50 border-blue-200'
                  }`}
                >
                  <div className="flex items-start justify-between">
                    <p className="text-[var(--cathay-text-primary)]">{alert.message}</p>
                    <Badge variant="outline" className="ml-2 border-[var(--cathay-jade-dark)] text-[var(--cathay-text-primary)]">{alert.time}</Badge>
                  </div>
                </div>
              ))}
            </div>
          </Card>

          {/* Recent Activities */}
          <Card className="bg-[var(--cathay-bg-card)] border border-[var(--cathay-border)] shadow-lg p-6">
            <div className="flex items-center mb-4">
              <MessageSquare className="w-5 h-5 text-[var(--cathay-jade-dark)] mr-2" />
              <h3 className="text-[var(--cathay-text-primary)] font-semibold">活动记录</h3>
            </div>
            <div className="space-y-3">
              {recentActivities.map(activity => (
                <div 
                  key={activity.id}
                  className="flex items-center justify-between p-4 bg-[var(--cathay-bg-subtle)] rounded-lg border border-[var(--cathay-border)]"
                >
                  <div>
                    <p className="text-[var(--cathay-text-primary)] font-medium">{activity.activity}</p>
                    <p className="text-[var(--cathay-text-secondary)]">{activity.time}</p>
                  </div>
                  <Badge variant="secondary" className="bg-[var(--cathay-border)] text-[var(--cathay-text-primary)]">{activity.duration}</Badge>
                </div>
              ))}
            </div>
          </Card>
        </div>
      </>
    );

  return (
    <div className="min-h-screen bg-[var(--cathay-bg-page)]">
      {/* Navigation */}
      <CrewNavigation 
        activeTab={activeTab} 
        onTabChange={setActiveTab}
        onLogout={onLogout}
      />
      
      <div className="max-w-7xl mx-auto p-6">
        {renderContent()}
      </div>
    </div>
  );
}
