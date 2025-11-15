import { User, Calendar, Phone, MapPin, Plane } from 'lucide-react';
import { Card } from '../ui/card';
import { Badge } from '../ui/badge';

interface ChildProfilePageProps {
  childName: string;
}

export function ChildProfilePage({ childName }: ChildProfilePageProps) {
  const childInfo = {
    name: childName,
    age: 8,
    gender: '男',
    birthday: '2017-03-15',
    guardianName: '李明',
    guardianPhone: '+86 138-0000-1234',
    emergencyContact: '张女士 +86 139-0000-5678',
    specialNeeds: '无',
    allergies: '海鲜过敏',
    medicalInfo: '无特殊医疗需求',
    travelHistory: [
      { date: '2024-12', destination: '东京', status: '已完成' },
      { date: '2024-08', destination: '新加坡', status: '已完成' },
      { date: '2024-03', destination: '悉尼', status: '已完成' }
    ]
  };

  return (
    <>
      {/* Header */}
      <div className="mb-6">
        <div className="bg-[var(--cathay-bg-card)] rounded-xl p-6 shadow-lg border border-[var(--cathay-border)]">
          <h1 className="text-[var(--cathay-text-primary)] mb-2 text-xl font-semibold">儿童档案</h1>
          <p className="text-[var(--cathay-text-secondary)]">查看和管理儿童基本信息</p>
        </div>
      </div>

      {/* Profile Header */}
      <Card className="bg-[var(--cathay-bg-card)] border border-[var(--cathay-border)] shadow-lg p-6 mb-6">
        <div className="flex items-start gap-6">
          <div className="w-24 h-24 rounded-full bg-[var(--cathay-bg-subtle)] flex items-center justify-center">
            <User className="w-12 h-12 text-[var(--cathay-jade-dark)]" />
          </div>
          <div className="flex-1">
            <div className="flex items-center gap-3 mb-2">
              <h2 className="text-[var(--cathay-text-primary)] text-lg font-semibold">{childInfo.name}</h2>
              <Badge className="bg-[var(--cathay-jade-dark)] text-white">单独旅行儿童</Badge>
            </div>
            <div className="grid grid-cols-2 gap-4 mt-4">
              <div>
                <span className="text-[var(--cathay-text-secondary)]">年龄：</span>
                <span className="text-[var(--cathay-text-primary)] font-medium">{childInfo.age}岁</span>
              </div>
              <div>
                <span className="text-[var(--cathay-text-secondary)]">性别：</span>
                <span className="text-[var(--cathay-text-primary)] font-medium">{childInfo.gender}</span>
              </div>
              <div>
                <span className="text-[var(--cathay-text-secondary)]">生日：</span>
                <span className="text-[var(--cathay-text-primary)] font-medium">{childInfo.birthday}</span>
              </div>
            </div>
          </div>
        </div>
      </Card>

      {/* Contact Information */}
      <Card className="bg-[var(--cathay-bg-card)] border border-[var(--cathay-border)] shadow-lg p-6 mb-6">
        <div className="flex items-center mb-4">
          <Phone className="w-5 h-5 text-[var(--cathay-jade-dark)] mr-2" />
          <h3 className="text-[var(--cathay-text-primary)] font-semibold text-lg">联系信息</h3>
        </div>
        <div className="space-y-4">
          <div className="bg-[var(--cathay-bg-subtle)] rounded-lg p-4 border border-[var(--cathay-border)]">
            <div className="text-[var(--cathay-text-secondary)] mb-1">监护人姓名</div>
            <div className="text-[var(--cathay-text-primary)] font-medium">{childInfo.guardianName}</div>
          </div>
          <div className="bg-[var(--cathay-bg-subtle)] rounded-lg p-4 border border-[var(--cathay-border)]">
            <div className="text-[var(--cathay-text-secondary)] mb-1">监护人电话</div>
            <div className="text-[var(--cathay-text-primary)] font-medium">{childInfo.guardianPhone}</div>
          </div>
          <div className="bg-[var(--cathay-bg-subtle)] rounded-lg p-4 border border-[var(--cathay-border)]">
            <div className="text-[var(--cathay-text-secondary)] mb-1">紧急联系人</div>
            <div className="text-[var(--cathay-text-primary)] font-medium">{childInfo.emergencyContact}</div>
          </div>
        </div>
      </Card>

      <div className="grid md:grid-cols-2 gap-6">
        {/* Medical Information */}
        <Card className="bg-[var(--cathay-bg-card)] border border-[var(--cathay-border)] shadow-lg p-6">
          <h3 className="text-[var(--cathay-text-primary)] font-semibold text-lg mb-4">医疗信息</h3>
          <div className="space-y-4">
            <div className="bg-[var(--cathay-bg-subtle)] rounded-lg p-4 border border-[var(--cathay-border)]">
              <div className="text-[var(--cathay-text-secondary)] mb-1">特殊需求</div>
              <div className="text-[var(--cathay-text-primary)] font-medium">{childInfo.specialNeeds}</div>
            </div>
            <div className="bg-amber-50 rounded-lg p-4 border border-amber-200">
              <div className="text-[var(--cathay-text-secondary)] mb-1">过敏信息</div>
              <div className="text-[var(--cathay-text-primary)] font-medium text-red-600">{childInfo.allergies}</div>
            </div>
            <div className="bg-[var(--cathay-bg-subtle)] rounded-lg p-4 border border-[var(--cathay-border)]">
              <div className="text-[var(--cathay-text-secondary)] mb-1">医疗备注</div>
              <div className="text-[var(--cathay-text-primary)] font-medium">{childInfo.medicalInfo}</div>
            </div>
          </div>
        </Card>

        {/* Travel History */}
        <Card className="bg-[var(--cathay-bg-card)] border border-[var(--cathay-border)] shadow-lg p-6">
          <div className="flex items-center mb-4">
            <Plane className="w-5 h-5 text-[var(--cathay-jade-dark)] mr-2" />
            <h3 className="text-[var(--cathay-text-primary)] font-semibold text-lg">旅行历史</h3>
          </div>
          <div className="space-y-4">
            {childInfo.travelHistory.map((trip, index) => (
              <div 
                key={index}
                className="flex items-center justify-between p-4 bg-[var(--cathay-bg-subtle)] rounded-lg border border-[var(--cathay-border)]"
              >
                <div className="flex items-center gap-3">
                  <MapPin className="w-4 h-4 text-[var(--cathay-jade-dark)]" />
                  <div>
                    <div className="text-[var(--cathay-text-primary)] font-medium">{trip.destination}</div>
                    <div className="text-[var(--cathay-text-secondary)] text-sm">{trip.date}</div>
                  </div>
                </div>
                <Badge variant="secondary" className="bg-[var(--cathay-border)] text-[var(--cathay-text-primary)]">
                  {trip.status}
                </Badge>
              </div>
            ))}
          </div>
        </Card>
      </div>
    </>
  );
}
