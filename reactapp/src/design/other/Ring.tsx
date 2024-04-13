import React from 'react';

interface RingIconProps {
  battery: number;
  radius?: number;
}

const RingIcon: React.FC<RingIconProps> = ({ radius = 10, battery }) => {
  const getStrokeColor = () => {
    if (battery <= 20) return 'red';
    if (battery <= 60) return 'yellow';
    return 'green';
  };

  const circumference = 2 * Math.PI * radius;
  const batteryStrokeDasharray = `${(circumference * battery) / 100} ${circumference}`;
  const fullChargeStrokeDasharray = `${circumference} ${circumference}`;

  return (
    <svg width={radius * 4} height={radius * 4} viewBox="0 0 50 50">
      <circle
        cx="25"
        cy="25"
        r={radius}
        fill="none"
        stroke="lightgrey"
        strokeWidth="5"
      />
      <circle
        cx="25"
        cy="25"
        r={radius}
        fill="none"
        stroke={getStrokeColor()}
        strokeWidth="5"
        strokeDasharray={batteryStrokeDasharray}
        transform="rotate(-90 25 25)"
        style={{
          animation: 'battery-fill 3s infinite ease-in-out',
        }}
      />
      <style>
        {`
          @keyframes battery-fill {
            0%, 100% {
              stroke-dasharray: ${batteryStrokeDasharray};
            }
            50% {
              stroke-dasharray: ${fullChargeStrokeDasharray};
            }
          }
        `}
      </style>
    </svg>
  );
};

export default RingIcon;
