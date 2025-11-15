import { useState } from 'react';
import { LoginPage } from './components/LoginPage';
import { CrewDashboard } from './components/crew/CrewDashboard';
import { ParentDashboard } from './components/parent/ParentDashboard';

type ViewType = 'login' | 'crew' | 'parent';

export default function App() {
  const [currentView, setCurrentView] = useState<ViewType>('login');
  const [childName] = useState('小明');

  const handleViewChange = (view: ViewType) => {
    setCurrentView(view);
  };

  const handleLogout = () => {
    setCurrentView('login');
  };

  return (
    <div className="min-h-screen bg-cathay-light-grey">
      {currentView === 'login' && (
        <LoginPage onSelectView={handleViewChange} />
      )}
      {currentView === 'crew' && (
        <CrewDashboard 
          childName={childName}
          onLogout={handleLogout}
        />
      )}
      {currentView === 'parent' && (
        <ParentDashboard 
          childName={childName}
          onLogout={handleLogout}
        />
      )}
    </div>
  );
}
