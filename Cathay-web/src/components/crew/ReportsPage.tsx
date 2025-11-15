import { BarChart3, TrendingUp, Users, Clock, Download } from 'lucide-react';
import { Card } from '../ui/card';
import { Button } from '../ui/button';
import { Badge } from '../ui/badge';

export function ReportsPage() {
  const flightStats = {
    totalChildren: 5,
    avgEmotionScore: 81.6,
    totalIncidents: 2,
    resolvedIncidents: 2,
    avgResponseTime: '3.5分钟',
    satisfactionRate: 98
  };

  const emotionTrend = [
    { time: '09:00', avg: 85 },
    { time: '10:00', avg: 83 },
    { time: '11:00', avg: 80 },
    { time: '12:00', avg: 78 },
    { time: '13:00', avg: 82 },
    { time: '14:00', avg: 85 }
  ];

  const childPerformance = [
    { name: '小明', emotionAvg: 85, incidents: 0, activities: 5 },
    { name: '小红', emotionAvg: 90, incidents: 0, activities: 4 },
    { name: '小华', emotionAvg: 65, incidents: 1, activities: 3 },
    { name: '小丽', emotionAvg: 80, incidents: 0, activities: 2 },
    { name: '小强', emotionAvg: 88, incidents: 1, activities: 6 }
  ];

  return (
    <>
      {/* Header */}
      <div className="mb-6">
        <div className="bg-[var(--cathay-bg-card)] rounded-xl p-6 shadow-lg border border-[var(--cathay-border)]">
          <div className="flex items-center justify-between">
            <div>
              <h1 className="text-[var(--cathay-text-primary)] mb-2 text-xl font-semibold">统计报告</h1>
              <p className="text-[var(--cathay-text-secondary)]">航班 CX889 · 香港 → 洛杉矶</p>
            </div>
            <Button className="bg-[var(--cathay-jade-dark)] text-white hover:bg-[var(--cathay-jade)]">
              <Download className="w-4 h-4 mr-2" />
              导出报告
            </Button>
          </div>
        </div>
      </div>

      {/* Key Metrics */}
      <div className="grid md:grid-cols-3 gap-6 mb-6">
        <Card className="bg-[var(--cathay-bg-card)] border border-[var(--cathay-border)] shadow-lg p-5">
          <div className="flex items-center justify-between mb-2">
            <div className="text-[var(--cathay-text-secondary)]">儿童总数</div>
            <Users className="w-5 h-5 text-[var(--cathay-jade-dark)]" />
          </div>
          <div className="text-[var(--cathay-text-primary)] text-3xl font-bold">{flightStats.totalChildren}</div>
          <div className="text-green-600 text-sm mt-1 flex items-center gap-1">
            <TrendingUp className="w-4 h-4" />
            全部在监护中
          </div>
        </Card>

        <Card className="bg-[var(--cathay-bg-card)] border border-[var(--cathay-border)] shadow-lg p-5">
          <div className="flex items-center justify-between mb-2">
            <div className="text-[var(--cathay-text-secondary)]">平均情绪评分</div>
            <BarChart3 className="w-5 h-5 text-[var(--cathay-jade-dark)]" />
          </div>
          <div className="text-[var(--cathay-text-primary)] text-3xl font-bold">{flightStats.avgEmotionScore}</div>
          <div className="text-green-600 text-sm mt-1">良好状态</div>
        </Card>

        <Card className="bg-[var(--cathay-bg-card)] border border-[var(--cathay-border)] shadow-lg p-5">
          <div className="flex items-center justify-between mb-2">
            <div className="text-[var(--cathay-text-secondary)]">事件响应时间</div>
            <Clock className="w-5 h-5 text-[var(--cathay-jade-dark)]" />
          </div>
          <div className="text-[var(--cathay-text-primary)] text-3xl font-bold">{flightStats.avgResponseTime}</div>
          <div className="text-green-600 text-sm mt-1">优秀表现</div>
        </Card>
      </div>

      {/* Emotion Trend Chart */}
      <Card className="bg-[var(--cathay-bg-card)] border border-[var(--cathay-border)] shadow-lg p-6 mb-6">
        <h3 className="text-[var(--cathay-text-primary)] font-semibold text-lg mb-4">情绪趋势分析</h3>
        <div className="space-y-3">
          {emotionTrend.map((point, index) => (
            <div key={index} className="flex items-center gap-4">
              <div className="text-[var(--cathay-text-secondary)] w-16 text-sm">{point.time}</div>
              <div className="flex-1">
                <div className="flex items-center gap-2">
                  <div className="flex-1 bg-[var(--cathay-bg-subtle)] rounded-full h-8 overflow-hidden">
                    <div 
                      className="bg-gradient-to-r from-[var(--cathay-jade-dark)] to-[var(--cathay-jade)] h-full flex items-center justify-end pr-3 text-white text-sm font-medium"
                      style={{ width: `${point.avg}%` }}
                    >
                      {point.avg}
                    </div>
                  </div>
                </div>
              </div>
            </div>
          ))}
        </div>
      </Card>

      {/* Individual Performance */}
      <Card className="bg-[var(--cathay-bg-card)] border border-[var(--cathay-border)] shadow-lg p-6 mb-6">
        <h3 className="text-[var(--cathay-text-primary)] font-semibold text-lg mb-4">个体表现统计</h3>
        <div className="space-y-4">
          {childPerformance.map((child, index) => (
            <div 
              key={index}
              className="bg-[var(--cathay-bg-subtle)] rounded-lg p-4 border border-[var(--cathay-border)]"
            >
              <div className="flex items-center justify-between mb-3">
                <div className="flex items-center gap-3">
                  <div className="w-10 h-10 rounded-full bg-[var(--cathay-jade-dark)] text-white flex items-center justify-center font-semibold">
                    {child.name[0]}
                  </div>
                  <div>
                    <div className="text-[var(--cathay-text-primary)] font-semibold">{child.name}</div>
                    <div className="text-[var(--cathay-text-secondary)] text-sm">
                      {child.activities} 项活动完成
                    </div>
                  </div>
                </div>
                <div className="flex items-center gap-3">
                  <Badge 
                    className={
                      child.emotionAvg >= 80 
                        ? 'bg-green-500 text-white'
                        : child.emotionAvg >= 60
                        ? 'bg-amber-500 text-white'
                        : 'bg-red-500 text-white'
                    }
                  >
                    情绪 {child.emotionAvg}
                  </Badge>
                  {child.incidents > 0 ? (
                    <Badge variant="outline" className="border-amber-500 text-amber-600">
                      {child.incidents} 次事件
                    </Badge>
                  ) : (
                    <Badge variant="outline" className="border-green-500 text-green-600">
                      无事件
                    </Badge>
                  )}
                </div>
              </div>
              <div className="w-full bg-gray-200 rounded-full h-2">
                <div 
                  className="bg-[var(--cathay-jade-dark)] h-2 rounded-full"
                  style={{ width: `${child.emotionAvg}%` }}
                ></div>
              </div>
            </div>
          ))}
        </div>
      </Card>

      {/* Summary */}
      <Card className="bg-gradient-to-r from-[var(--cathay-jade-dark)] to-[var(--cathay-jade)] text-white border-0 shadow-lg p-6">
        <div className="flex items-center justify-between">
          <div>
            <h3 className="text-xl font-semibold mb-2">服务质量评分</h3>
            <p className="opacity-90">基于家长反馈和系统监测数据</p>
          </div>
          <div className="text-right">
            <div className="text-5xl font-bold">{flightStats.satisfactionRate}%</div>
            <div className="text-sm opacity-90">满意度</div>
          </div>
        </div>
      </Card>
    </>
  );
}
