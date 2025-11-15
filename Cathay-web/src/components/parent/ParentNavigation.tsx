import { Home, User, Bell, Settings, LogOut } from 'lucide-react';

const cathayLogo = new URL('../../assets/Cathay Pacific US_idC8zQCjm4_2.svg', import.meta.url).href;

interface ParentNavigationProps {
  activeTab: string;
  onTabChange: (tab: string) => void;
  onLogout: () => void;
}

export function ParentNavigation({ activeTab, onTabChange, onLogout }: ParentNavigationProps) {
  const navItems = [
    { id: 'home', label: '首页', icon: Home },
    { id: 'profile', label: '儿童信息', icon: User },
    { id: 'notifications', label: '通知', icon: Bell },
    { id: 'settings', label: '设置', icon: Settings },
  ];

  return (
    <nav className="bg-[var(--cathay-jade-dark)] border-b border-[var(--cathay-jade)] shadow-sm sticky top-0 z-50">
      <div className="max-w-5xl mx-auto px-6">
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
