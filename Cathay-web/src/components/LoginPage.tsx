import React from 'react';
import { UserRound, Users } from 'lucide-react';
import { Button } from './ui/button';
import { Card } from './ui/card';
import { CathayLogo } from './CathayLogo';

interface LoginPageProps {
  onSelectView: (view: 'crew' | 'parent') => void;
}

export function LoginPage({ onSelectView }: LoginPageProps) {
  return (
    <div className="min-h-screen flex items-center justify-center p-6 bg-[var(--cathay-bg-card)]">
      <div className="w-full max-w-4xl">
        {/* Logo区域 - 遵循品牌指南，水平Logo居中显示 */}
        <div className="text-center mb-12">
          <div className="flex items-center justify-center mb-6">
            <CathayLogo variant="horizontal" color="jade" size="md" />
          </div>
          <h2 className="text-[var(--cathay-text-secondary)]">儿童智能旅行监护系统</h2>
          <p className="text-[var(--cathay-text-secondary)]">Children's Smart Travel Guardian</p>
        </div>

        <div className="grid md:grid-cols-2 gap-6">
          <Card 
            className="bg-[var(--cathay-bg-card)] border border-[var(--cathay-border)] shadow-lg hover:shadow-xl transition-all cursor-pointer p-8"
            onClick={() => onSelectView('crew')}
          >
            <div className="text-center">
              <div className="bg-[var(--cathay-jade-dark)] p-6 rounded-xl inline-flex mb-6">
                <Users className="w-12 h-12 text-black" />
              </div>
              <h2 className="text-[var(--cathay-text-primary)] mb-3 text-xl font-semibold">乘务员端</h2>
              <p className="text-[var(--cathay-text-secondary)] mb-6">
                实时监控儿童情绪、位置和健康状态，确保旅程安全
              </p>
              <Button 
                className="bg-[var(--cathay-bg-card)] hover:bg-[var(--cathay-bg-subtle)] text-[var(--cathay-text-dark)] border border-[var(--cathay-jade-dark)] w-full"
                size="lg"
              >
                进入乘务员系统
              </Button>
            </div>
          </Card>

          <Card 
            className="bg-[var(--cathay-bg-card)] border border-[var(--cathay-border)] shadow-lg hover:shadow-xl transition-all cursor-pointer p-8"
            onClick={() => onSelectView('parent')}
          >
            <div className="text-center">
              <div className="bg-[var(--cathay-jade)] p-6 rounded-xl inline-flex mb-6">
                <UserRound className="w-12 h-12 text-black" />
              </div>
              <h2 className="text-[var(--cathay-text-primary)] mb-3 text-xl font-semibold">家长端</h2>
              <p className="text-[var(--cathay-text-secondary)] mb-6">
                查看孩子的旅行进度和基本状态，随时了解旅程情况
              </p>
              <Button 
                className="bg-[var(--cathay-bg-card)] hover:bg-[var(--cathay-bg-subtle)] text-[var(--cathay-text-dark)] border border-[var(--cathay-jade)] w-full"
                size="lg"
              >
                进入家长系统
              </Button>
            </div>
          </Card>
        </div>

      </div>
    </div>
  );
}
