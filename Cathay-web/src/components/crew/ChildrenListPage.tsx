import { Users, Search, Filter } from 'lucide-react';
import { Card } from '../ui/card';
import { Badge } from '../ui/badge';
import { Button } from '../ui/button';
import { Input } from '../ui/input';

export function ChildrenListPage() {
  const children = [
    {
      id: 1,
      name: '小明',
      age: 8,
      seat: '12A',
      status: 'normal',
      emotion: '开心',
      emotionScore: 85,
      lastActivity: '观看动画片',
      heartRate: 78,
      temperature: 36.5
    },
    {
      id: 2,
      name: '小红',
      age: 7,
      seat: '15B',
      status: 'normal',
      emotion: '愉快',
      emotionScore: 90,
      lastActivity: '玩游戏',
      heartRate: 75,
      temperature: 36.6
    },
    {
      id: 3,
      name: '小华',
      age: 9,
      seat: '18C',
      status: 'attention',
      emotion: '想家',
      emotionScore: 65,
      lastActivity: '与AI玩偶对话',
      heartRate: 82,
      temperature: 36.4
    },
    {
      id: 4,
      name: '小丽',
      age: 6,
      seat: '20D',
      status: 'normal',
      emotion: '平静',
      emotionScore: 80,
      lastActivity: '午睡',
      heartRate: 70,
      temperature: 36.5
    },
    {
      id: 5,
      name: '小强',
      age: 10,
      seat: '22A',
      status: 'normal',
      emotion: '兴奋',
      emotionScore: 88,
      lastActivity: '阅读',
      heartRate: 80,
      temperature: 36.7
    }
  ];

  const getStatusBadge = (status: string) => {
    switch (status) {
      case 'normal':
        return <Badge className="bg-green-500 text-white">正常</Badge>;
      case 'attention':
        return <Badge className="bg-amber-500 text-white">需关注</Badge>;
      case 'alert':
        return <Badge className="bg-red-500 text-white">警告</Badge>;
      default:
        return <Badge className="bg-gray-500 text-white">未知</Badge>;
    }
  };

  const getEmotionColor = (score: number) => {
    if (score >= 80) return 'text-green-600';
    if (score >= 60) return 'text-amber-600';
    return 'text-red-600';
  };

  return (
    <>
      {/* Header */}
      <div className="mb-6">
        <div className="bg-[var(--cathay-bg-card)] rounded-xl p-6 shadow-lg border border-[var(--cathay-border)]">
          <div className="flex items-center justify-between mb-4">
            <div>
              <h1 className="text-[var(--cathay-text-primary)] mb-2 text-xl font-semibold">儿童列表</h1>
              <p className="text-[var(--cathay-text-secondary)]">共 {children.length} 名单独旅行儿童</p>
            </div>

          </div>
          <div className="relative">  
            <Input 
              placeholder="搜索儿童姓名或座位号..." 
              className="pl-10 border-[var(--cathay-border)] bg-[var(--cathay-bg-subtle)]"
            />
          </div>
        </div>
      </div>

      {/* Children Grid */}
      <div className="grid md:grid-cols-2 gap-6">
        {children.map((child) => (
          <Card 
            key={child.id}
            className="bg-[var(--cathay-bg-card)] border border-[var(--cathay-border)] shadow-lg p-6 hover:shadow-xl transition-shadow cursor-pointer"
          >
            <div className="flex items-start justify-between mb-4">
              <div>
                <h3 className="text-[var(--cathay-text-primary)] text-lg font-semibold">{child.name}</h3>
                <p className="text-[var(--cathay-text-secondary)]">{child.age}岁 · 座位 {child.seat}</p>
              </div>
              {getStatusBadge(child.status)}
            </div>

            <div className="grid grid-cols-2 gap-4 mb-4">
              <div className="bg-[var(--cathay-bg-subtle)] rounded-lg p-3 border border-[var(--cathay-border)]">
                <div className="text-[var(--cathay-text-secondary)] text-sm mb-1">当前情绪</div>
                <div className={`font-semibold ${getEmotionColor(child.emotionScore)}`}>
                  {child.emotion} ({child.emotionScore})
                </div>
              </div>
              <div className="bg-[var(--cathay-bg-subtle)] rounded-lg p-3 border border-[var(--cathay-border)]">
                <div className="text-[var(--cathay-text-secondary)] text-sm mb-1">心率</div>
                <div className="text-[var(--cathay-text-primary)] font-semibold">{child.heartRate} BPM</div>
              </div>
            </div>

            <div className="bg-blue-50 border border-blue-200 rounded-lg p-3 mb-3">
              <div className="text-[var(--cathay-text-secondary)] text-sm mb-1">当前活动</div>
              <div className="text-[var(--cathay-text-primary)] font-medium">{child.lastActivity}</div>
            </div>

            <Button 
              size="sm" 
              className="w-full bg-[var(--cathay-jade-dark)] text-black hover:bg-[var(--cathay-jade)]"
            >
              查看详情
            </Button>
          </Card>
        ))}
      </div>
    </>
  );
}
