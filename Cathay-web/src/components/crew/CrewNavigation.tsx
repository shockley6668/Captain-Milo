import { LayoutDashboard, Users, AlertTriangle, BarChart3, Settings, LogOut } from 'lucide-react';

const cathayLogo = new URL('../../assets/Cathay Pacific US_idC8zQCjm4_2.svg', import.meta.url).href;

interface CrewNavigationProps {
  activeTab: string;
  onTabChange: (tab: string) => void;
  onLogout: () => void;
}

export function CrewNavigation({ activeTab, onTabChange, onLogout }: CrewNavigationProps) {
  const navItems = [
    { id: 'dashboard', label: '监控面板', icon: LayoutDashboard },
    { id: 'children', label: '儿童列表', icon: Users },
    { id: 'alerts', label: '提醒中心', icon: AlertTriangle },
    { id: 'reports', label: '报告', icon: BarChart3 },
    { id: 'settings', label: '设置', icon: Settings },
  ];

  return (
    <nav className="bg-[var(--cathay-jade-dark)] border-b border-[var(--cathay-jade)] shadow-sm sticky top-0 z-50">
      <div className="max-w-7xl mx-auto px-6">
        <div className="flex items-center justify-between h-16">
          {/* Logo */}
          <div className="flex items-center gap-4">
            <img 
              src={cathayLogo} 
              alt="Cathay Pacific" 
              className="h-8" 
            />
          </div>

          {/* Navigation Items */}
          <div className="flex items-center gap-2">
            {navItems.map((item) => {
              const Icon = item.icon;
              return (
                <button
                  key={item.id}
                  onClick={() => onTabChange(item.id)}
                  className="flex items-center gap-2 px-4 py-2 rounded-lg transition-colors text-white/80 hover:bg-white/10 hover:text-white"
                >
                  <Icon className="w-4 h-4" />
                  <span className="text-sm font-medium">{item.label}</span>
                </button>
              );
            })}
            
            {/* Logout Button */}
            <button
              onClick={onLogout}
              className="flex items-center gap-2 px-4 py-2 rounded-lg transition-colors text-white/80 hover:bg-red-500/20 hover:text-white ml-2"
              title="登出"
            >
              <LogOut className="w-4 h-4" />
              <span className="text-sm font-medium">登出</span>
            </button>
          </div>
        </div>
      </div>
    </nav>
  );
}
