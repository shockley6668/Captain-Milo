import { useState } from 'react';
import { LoginPage } from './components/LoginPage';
import { CrewDashboard } from './components/CrewDashboard';
import { ParentDashboard } from './components/ParentDashboard';

type ViewType = 'login' | 'crew' | 'parent';

export default function App() {
  const [currentView, setCurrentView] = useState<ViewType>('login');
  const [childName] = useState('小明');

  const handleViewChange = (view: ViewType) => {
    setCurrentView(view);
  };

  return (
    <div className="min-h-screen bg-[#f5f7f7]">
      {currentView === 'login' && (
        <LoginPage onSelectView={handleViewChange} />
      )}
      {currentView === 'crew' && (
        <CrewDashboard 
          childName={childName} 
          onBack={() => handleViewChange('login')} 
        />
      )}
      {currentView === 'parent' && (
        <ParentDashboard 
          childName={childName} 
          onBack={() => handleViewChange('login')} 
        />
      )}
    </div>
  );
}
