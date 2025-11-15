import { BarChart, Bar, XAxis, YAxis, CartesianGrid, Tooltip, ResponsiveContainer } from 'recharts';

export function TravelProgressChart() {
  const data = [
    { stage: '登机', progress: 100 },
    { stage: '起飞', progress: 100 },
    { stage: '飞行', progress: 45 },
    { stage: '降落', progress: 0 },
    { stage: '到达', progress: 0 }
  ];

  return (
    <ResponsiveContainer width="100%" height={200}>
      <BarChart data={data}>
        <CartesianGrid strokeDasharray="3 3" stroke="#e0e0e0" />
        <XAxis 
          dataKey="stage" 
          stroke="#8b9088"
          style={{ fontSize: '12px' }}
        />
        <YAxis 
          stroke="#8b9088"
          style={{ fontSize: '12px' }}
          domain={[0, 100]}
        />
        <Tooltip 
          contentStyle={{ 
            backgroundColor: 'white', 
            border: 'none',
            borderRadius: '8px',
            boxShadow: '0 4px 6px rgba(0,0,0,0.1)'
          }}
        />
        <Bar 
          dataKey="progress" 
          fill="#a4b5a8"
          radius={[8, 8, 0, 0]}
        />
      </BarChart>
    </ResponsiveContainer>
  );
}
