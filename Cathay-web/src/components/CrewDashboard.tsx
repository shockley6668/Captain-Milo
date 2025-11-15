import { ArrowLeft, MapPin, Heart, Thermometer, Activity, AlertCircle, MessageSquare, Clock } from 'lucide-react';
import { Button } from './ui/button';
import { Card } from './ui/card';
import { Badge } from './ui/badge';
import { Progress } from './ui/progress';
import { EmotionChart } from './EmotionChart';
import { VitalSignsChart } from './VitalSignsChart';
import { CathayLogo } from './CathayLogo';

interface CrewDashboardProps {
  childName: string;
  onBack: () => void;
}

export function CrewDashboard({ childName, onBack }: CrewDashboardProps) {
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

  return (
    <div className="min-h-screen p-6 bg-[#f5f7f7]">
      <div className="max-w-7xl mx-auto">
        {/* Header with Logo */}
        <div className="mb-6">
          <div className="flex items-center justify-between mb-4">
            <Button 
              variant="ghost" 
              onClick={onBack}
              className="text-[#005d63] hover:text-[#004a4f] hover:bg-white/50"
            >
              <ArrowLeft className="w-4 h-4 mr-2" />
              返回
            </Button>
            <CathayLogo variant="horizontal" color="jade" size="md" />
          </div>
          <div className="flex items-center justify-between bg-white rounded-xl p-6 shadow-lg border border-[#e0e8e9]">
            <div>
              <h1 className="text-[#005d63] mb-2 text-xl font-semibold">乘务员监控系统</h1>
              <p className="text-[#4a6b6f]">航班 {childInfo.flight} · {childInfo.destination}</p>
            </div>
            <div className="text-right">
              <div className="text-[#4a6b6f]">最后更新</div>
              <div className="text-[#005d63] flex items-center justify-end">
                <Clock className="w-4 h-4 mr-1" />
                {childInfo.lastUpdate}
              </div>
            </div>
          </div>
        </div>

        {/* Child Info Card */}
        <Card className="bg-white border border-[#e0e8e9] shadow-lg p-6 mb-6">
          <div className="flex items-start justify-between mb-6">
            <div>
              <h2 className="text-[#005d63] mb-2 text-lg font-semibold">儿童信息</h2>
              <div className="space-y-2">
                <p className="text-[#4a6b6f]">姓名：{childInfo.name} · {childInfo.age}岁</p>
                <p className="text-[#4a6b6f]">座位：{childInfo.seat}</p>
              </div>
            </div>
            <Badge className="bg-[#005d63] text-white">状态正常</Badge>
          </div>

          <div className="grid md:grid-cols-4 gap-4">
            <div className="bg-[#f0f5f6] rounded-lg p-4 border border-[#e0e8e9]">
              <div className="flex items-center mb-2">
                <Activity className="w-5 h-5 text-[#005d63] mr-2" />
                <span className="text-[#4a6b6f]">当前情绪</span>
              </div>
              <div className="text-[#005d63] font-medium">{childInfo.currentEmotion}</div>
              <Progress value={childInfo.emotionScore} className="mt-2" />
            </div>

            <div className="bg-[#f0f5f6] rounded-lg p-4 border border-[#e0e8e9]">
              <div className="flex items-center mb-2">
                <MapPin className="w-5 h-5 text-[#005d63] mr-2" />
                <span className="text-[#4a6b6f]">位置</span>
              </div>
              <div className="text-[#005d63] font-medium">{childInfo.location}</div>
            </div>

            <div className="bg-[#f0f5f6] rounded-lg p-4 border border-[#e0e8e9]">
              <div className="flex items-center mb-2">
                <Heart className="w-5 h-5 text-[#005d63] mr-2" />
                <span className="text-[#4a6b6f]">心率</span>
              </div>
              <div className="text-[#005d63] font-medium">{childInfo.heartRate} BPM</div>
            </div>

            <div className="bg-[#f0f5f6] rounded-lg p-4 border border-[#e0e8e9]">
              <div className="flex items-center mb-2">
                <Thermometer className="w-5 h-5 text-[#005d63] mr-2" />
                <span className="text-[#4a6b6f]">体温</span>
              </div>
              <div className="text-[#005d63] font-medium">{childInfo.temperature}°C</div>
            </div>
          </div>
        </Card>

        <div className="grid md:grid-cols-2 gap-6 mb-6">
          {/* Emotion Chart */}
          <Card className="bg-white border border-[#e0e8e9] shadow-lg p-6">
            <h3 className="text-[#005d63] mb-4 font-semibold">情绪趋势分析</h3>
            <EmotionChart />
          </Card>

          {/* Vital Signs Chart */}
          <Card className="bg-white border border-[#e0e8e9] shadow-lg p-6">
            <h3 className="text-[#005d63] mb-4 font-semibold">生命体征监测</h3>
            <VitalSignsChart />
          </Card>
        </div>

        <div className="grid md:grid-cols-2 gap-6">
          {/* Alerts */}
          <Card className="bg-white border border-[#e0e8e9] shadow-lg p-6">
            <div className="flex items-center mb-4">
              <AlertCircle className="w-5 h-5 text-[#005d63] mr-2" />
              <h3 className="text-[#005d63] font-semibold">提醒事项</h3>
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
                    <p className="text-[#005d63]">{alert.message}</p>
                    <Badge variant="outline" className="ml-2 border-[#005d63] text-[#005d63]">{alert.time}</Badge>
                  </div>
                </div>
              ))}
            </div>
          </Card>

          {/* Recent Activities */}
          <Card className="bg-white border border-[#e0e8e9] shadow-lg p-6">
            <div className="flex items-center mb-4">
              <MessageSquare className="w-5 h-5 text-[#005d63] mr-2" />
              <h3 className="text-[#005d63] font-semibold">活动记录</h3>
            </div>
            <div className="space-y-3">
              {recentActivities.map(activity => (
                <div 
                  key={activity.id}
                  className="flex items-center justify-between p-4 bg-[#f0f5f6] rounded-lg border border-[#e0e8e9]"
                >
                  <div>
                    <p className="text-[#005d63] font-medium">{activity.activity}</p>
                    <p className="text-[#4a6b6f]">{activity.time}</p>
                  </div>
                  <Badge variant="secondary" className="bg-[#e0e8e9] text-[#005d63]">{activity.duration}</Badge>
                </div>
              ))}
            </div>
          </Card>
        </div>
      </div>
    </div>
  );
}
