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
  
  // Logo尺寸配置 - 响应式高度控制，根据视口大小调整
  const logoHeight = {
    sm: 'h-6 sm:h-7 md:h-8', // 移动端24px -> 平板28px -> 桌面32px
    md: 'h-8 sm:h-10 md:h-12 lg:h-14', // 移动端32px -> 平板40px -> 桌面48px -> 大屏56px
    lg: 'h-10 sm:h-12 md:h-14 lg:h-16 xl:h-20'  // 移动端40px -> 平板48px -> 桌面56px -> 大屏64px -> 超大屏80px
  };

  // 排除区域：0.5x brushwing图标高度，使用padding实现
  // 品牌指南要求：必须在Logo周围保持0.5x的排除区域
  // 响应式padding，根据视口大小调整
  const exclusionZone = {
    sm: 'p-2 sm:p-2.5 md:p-3', // 移动端8px -> 平板10px -> 桌面12px
    md: 'p-2 sm:p-3 md:p-4 lg:p-5', // 移动端8px -> 平板12px -> 桌面16px -> 大屏20px
    lg: 'p-3 sm:p-4 md:p-5 lg:p-6 xl:p-8'  // 移动端12px -> 平板16px -> 桌面20px -> 大屏24px -> 超大屏32px
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

