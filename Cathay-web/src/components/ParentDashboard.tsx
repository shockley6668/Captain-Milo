import { ArrowLeft, MapPin, Smile, Plane, Clock } from 'lucide-react';
import { Button } from './ui/button';
import { Card } from './ui/card';
import { Badge } from './ui/badge';
import { Progress } from './ui/progress';
import { TravelProgressChart } from './TravelProgressChart';
import { CathayLogo } from './CathayLogo';

interface ParentDashboardProps {
  childName: string;
  onBack: () => void;
}

export function ParentDashboard({ childName, onBack }: ParentDashboardProps) {
  // 模拟数据
  const travelInfo = {
    name: childName,
    flight: 'CX889',
    from: '香港',
    to: '洛杉矶',
    departure: '09:30',
    arrival: '06:45',
    progress: 45,
    currentStatus: '飞行中',
    emotion: '开心愉快',
    lastActivity: '正在观看动画片',
    estimatedRemaining: '8小时30分钟'
  };

  const milestones = [
    { id: 1, title: '登机完成', time: '09:15', status: 'completed' },
    { id: 2, title: '起飞', time: '09:45', status: 'completed' },
    { id: 3, title: '午餐时间', time: '11:30', status: 'completed' },
    { id: 4, title: '娱乐时间', time: '12:00', status: 'active' },
    { id: 5, title: '休息时间', time: '14:00', status: 'upcoming' },
    { id: 6, title: '到达目的地', time: '06:45', status: 'upcoming' }
  ];

  return (
    <div className="min-h-screen p-6 bg-[#f5f7f7]">
      <div className="max-w-5xl mx-auto">
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
          <div className="bg-white rounded-xl p-6 shadow-lg border border-[#e0e8e9]">
            <h1 className="text-[#005d63] mb-2 text-xl font-semibold">家长监护系统</h1>
            <p className="text-[#4a6b6f]">您的孩子 {childName} 正在安全旅行中</p>
          </div>
        </div>

        {/* Flight Progress */}
        <Card className="bg-white border border-[#e0e8e9] shadow-lg p-6 mb-6">
          <div className="flex items-center justify-between mb-6">
            <h2 className="text-[#005d63] text-lg font-semibold">旅行进度</h2>
            <Badge className="bg-[#005d63] text-white">{travelInfo.currentStatus}</Badge>
          </div>
          
          <div className="mb-6">
            <div className="flex items-center justify-between mb-4">
              <div className="flex items-center">
                <div className="text-black rounded-lg px-4 py-2 mr-4 font-medium">
                  {travelInfo.from}
                </div>
                <Plane className="w-6 h-6 text-[#005d63] rotate-90" />
              </div>
              <div className="flex-1 mx-4">
                <Progress value={travelInfo.progress} className="h-2" />
                <div className="text-center mt-2 text-[#4a6b6f]">
                  {travelInfo.progress}% 已完成
                </div>
              </div>
              <div className="flex items-center">
                <Plane className="w-6 h-6 text-[#007a82] rotate-90" />
                <div className="text-black rounded-lg px-4 py-2 ml-4 font-medium">
                  {travelInfo.to}
                </div>
              </div>
            </div>
            
            <div className="grid grid-cols-2 gap-4 mt-6">
              <div className="bg-[#f0f5f6] rounded-lg p-4 border border-[#e0e8e9]">
                <div className="text-[#4a6b6f] mb-1">出发时间</div>
                <div className="text-[#005d63] font-medium">{travelInfo.departure}</div>
              </div>
              <div className="bg-[#f0f5f6] rounded-lg p-4 border border-[#e0e8e9]">
                <div className="text-[#4a6b6f] mb-1">预计到达</div>
                <div className="text-[#005d63] font-medium">{travelInfo.arrival}</div>
              </div>
            </div>
          </div>

          <div className="bg-blue-50 rounded-lg p-4 flex items-center border border-blue-200">
            <Clock className="w-5 h-5 text-[#005d63] mr-3" />
            <div>
              <div className="text-[#005d63] font-medium">预计剩余时间</div>
              <div className="text-[#4a6b6f]">{travelInfo.estimatedRemaining}</div>
            </div>
          </div>
        </Card>

        {/* Current Status */}
        <div className="grid md:grid-cols-2 gap-6 mb-6">
          <Card className="bg-white border border-[#e0e8e9] shadow-lg p-6">
            <div className="flex items-center mb-4">
              <Smile className="w-6 h-6 text-[#005d63] mr-2" />
              <h3 className="text-[#005d63] font-semibold">当前状态</h3>
            </div>
            <div className="space-y-4">
              <div className="bg-green-50 rounded-lg p-4 border border-green-200">
                <div className="text-[#4a6b6f] mb-1">情绪状态</div>
                <div className="text-[#005d63] font-medium">{travelInfo.emotion}</div>
              </div>
              <div className="bg-[#f0f5f6] rounded-lg p-4 border border-[#e0e8e9]">
                <div className="text-[#4a6b6f] mb-1">当前活动</div>
                <div className="text-[#005d63] font-medium">{travelInfo.lastActivity}</div>
              </div>
            </div>
          </Card>

          <Card className="bg-white border border-[#e0e8e9] shadow-lg p-6">
            <div className="flex items-center mb-4">
              <MapPin className="w-6 h-6 text-[#005d63] mr-2" />
              <h3 className="text-[#005d63] font-semibold">航班信息</h3>
            </div>
            <div className="space-y-4">
              <div className="bg-[#f0f5f6] rounded-lg p-4 border border-[#e0e8e9]">
                <div className="text-[#4a6b6f] mb-1">航班号</div>
                <div className="text-[#005d63] font-medium">{travelInfo.flight}</div>
              </div>
              <div className="bg-[#f0f5f6] rounded-lg p-4 border border-[#e0e8e9]">
                <div className="text-[#4a6b6f] mb-1">航线</div>
                <div className="text-[#005d63] font-medium">{travelInfo.from} → {travelInfo.to}</div>
              </div>
            </div>
          </Card>
        </div>

        {/* Travel Milestones */}
        <Card className="bg-white border border-[#e0e8e9] shadow-lg p-6">
          <h3 className="text-[#005d63] mb-6 font-semibold">旅程里程碑</h3>
          <div className="space-y-4">
            {milestones.map((milestone, index) => (
              <div key={milestone.id} className="flex items-center">
                <div className="relative">
                  <div 
                    className={`w-10 h-10 rounded-full flex items-center justify-center ${
                      milestone.status === 'completed' 
                        ? 'bg-[#005d63]' 
                        : milestone.status === 'active'
                        ? 'bg-[#007a82]'
                        : 'bg-gray-300'
                    }`}
                  >
                    <div className="w-4 h-4 bg-white rounded-full"></div>
                  </div>
                  {index < milestones.length - 1 && (
                    <div className="absolute left-1/2 top-10 w-0.5 h-8 -ml-px bg-gray-300"></div>
                  )}
                </div>
                <div className="ml-4 flex-1">
                  <div className="flex items-center justify-between">
                    <div>
                      <p className="text-[#005d63] font-medium">{milestone.title}</p>
                      <p className="text-[#4a6b6f]">{milestone.time}</p>
                    </div>
                    <Badge 
                      variant={
                        milestone.status === 'completed' 
                          ? 'default' 
                          : milestone.status === 'active'
                          ? 'default'
                          : 'secondary'
                      }
                      className={
                        milestone.status === 'completed' 
                          ? 'bg-[#005d63] text-white' 
                          : milestone.status === 'active'
                          ? 'bg-[#007a82] text-white'
                          : 'bg-gray-300 text-gray-600'
                      }
                    >
                      {milestone.status === 'completed' 
                        ? '已完成' 
                        : milestone.status === 'active'
                        ? '进行中'
                        : '待完成'}
                    </Badge>
                  </div>
                </div>
              </div>
            ))}
          </div>
        </Card>

        {/* AI Companion Info */}
        <Card className="bg-gradient-to-r from-[#005d63] to-[#007a82] text-white border-0 shadow-lg p-6 mt-6">
          <h3 className="mb-3 font-semibold">AI智能陪伴</h3>
          <p className="opacity-90">
            智能玩偶正在陪伴您的孩子，实时监测情绪变化，提供互动娱乐，确保旅程愉快安全。
          </p>
        </Card>
      </div>
    </div>
  );
}
