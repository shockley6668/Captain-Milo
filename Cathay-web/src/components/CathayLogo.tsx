import React from 'react';

// 使用Vite的静态资源导入方式
// 注意：如果TypeScript报错，请确保vite-env.d.ts文件已正确配置
const cathayLogoJade = new URL('../assets/Cathay Pacific US_idC8zQCjm4_3.png', import.meta.url).href;

interface CathayLogoProps {
  variant?: 'horizontal' | 'vertical';
  color?: 'jade' | 'white';
  size?: 'sm' | 'md' | 'lg';
  className?: string;
}

export function CathayLogo({ 
  variant = 'horizontal', 
  color = 'jade',
  size = 'md',
  className = ''
}: CathayLogoProps) {
  // 根据品牌指南，Logo周围需要0.5x的排除区域（x是brushwing图标的高度）
  // 这里我们使用padding来实现排除区域
  
  // Logo尺寸配置 - 高度控制
  const logoHeight = {
    sm: 'h-8', // 最小高度约32px，符合品牌指南最小尺寸要求
    md: 'h-12', // 48px
    lg: 'h-16'  // 64px
  };

  // 排除区域：0.5x brushwing图标高度，使用padding实现
  // 品牌指南要求：必须在Logo周围保持0.5x的排除区域
  const exclusionZone = {
    sm: 'p-3', // 约12px padding (0.5 * 24px)
    md: 'p-4', // 约16px padding (0.5 * 32px)
    lg: 'p-6'  // 约24px padding (0.5 * 48px)
  };

  // 根据颜色选择Logo变体
  // 品牌指南：浅色背景使用Cathay Jade版本，深色背景使用白色版本
  // 注意：如果图片是Cathay Jade版本，在深色背景上可能需要使用CSS滤镜转换为白色
  const logoStyle = color === 'white' 
    ? { filter: 'brightness(0) invert(1)' } // 将Logo转换为白色
    : {};

  // 垂直Logo（仅在水平空间受限时使用）
  if (variant === 'vertical') {
    return (
      <div className={`flex flex-col items-center ${exclusionZone[size]} ${className}`}>
        <img 
          src={cathayLogoJade} 
          alt="Cathay Pacific" 
          className={`${logoHeight[size]} w-auto object-contain`}
          style={logoStyle}
        />
      </div>
    );
  }

  // 水平Logo（默认，符合品牌指南要求）
  // 品牌指南：水平Logo是首选，应在所有通讯中优先使用
  return (
    <div className={`flex items-center ${exclusionZone[size]} ${className}`}>
      <img 
        src={cathayLogoJade} 
        alt="Cathay Pacific" 
        className={`${logoHeight[size]} w-auto object-contain`}
        style={logoStyle}
      />
    </div>
  );
}

