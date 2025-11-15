import { LineChart, Line, XAxis, YAxis, CartesianGrid, Tooltip, ResponsiveContainer } from 'recharts';

export function EmotionChart() {
  const data = [
    { time: '09:00', score: 75 },
    { time: '10:00', score: 82 },
    { time: '11:00', score: 78 },
    { time: '12:00', score: 85 },
    { time: '13:00', score: 88 },
    { time: '14:00', score: 85 }
  ];

  return (
    <ResponsiveContainer width="100%" height={200}>
      <LineChart data={data}>
        <CartesianGrid strokeDasharray="3 3" stroke="#e0e0e0" />
        <XAxis 
          dataKey="time" 
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
        <Line 
          type="monotone" 
          dataKey="score" 
          stroke="#6b7a6f" 
          strokeWidth={3}
          dot={{ fill: '#6b7a6f', r: 4 }}
          activeDot={{ r: 6 }}
        />
      </LineChart>
    </ResponsiveContainer>
  );
}
