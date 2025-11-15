import { AreaChart, Area, XAxis, YAxis, CartesianGrid, Tooltip, ResponsiveContainer } from 'recharts';

export function VitalSignsChart() {
  const data = [
    { time: '09:00', heartRate: 75, temp: 36.4 },
    { time: '10:00', heartRate: 78, temp: 36.5 },
    { time: '11:00', heartRate: 80, temp: 36.6 },
    { time: '12:00', heartRate: 76, temp: 36.5 },
    { time: '13:00', heartRate: 74, temp: 36.4 },
    { time: '14:00', heartRate: 78, temp: 36.5 }
  ];

  return (
    <ResponsiveContainer width="100%" height={200}>
      <AreaChart data={data}>
        <CartesianGrid strokeDasharray="3 3" stroke="#e0e0e0" />
        <XAxis 
          dataKey="time" 
          stroke="#8b9088"
          style={{ fontSize: '12px' }}
        />
        <YAxis 
          stroke="#8b9088"
          style={{ fontSize: '12px' }}
          domain={[70, 85]}
        />
        <Tooltip 
          contentStyle={{ 
            backgroundColor: 'white', 
            border: 'none',
            borderRadius: '8px',
            boxShadow: '0 4px 6px rgba(0,0,0,0.1)'
          }}
        />
        <Area 
          type="monotone" 
          dataKey="heartRate" 
          stroke="#6b7a6f" 
          fill="#a4b5a8"
          fillOpacity={0.6}
          strokeWidth={2}
        />
      </AreaChart>
    </ResponsiveContainer>
  );
}
