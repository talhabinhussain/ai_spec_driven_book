import React, { PropsWithChildren, useEffect, useState } from 'react';

interface ResponsiveContainerProps {
  maxWidth?: number;
  padding?: string;
  className?: string;
  centerContent?: boolean;
}

const ResponsiveContainer: React.FC<PropsWithChildren<ResponsiveContainerProps>> = ({
  maxWidth = 1200,
  padding = '20px',
  className = '',
  centerContent = true,
  children
}) => {
  const [windowWidth, setWindowWidth] = useState(typeof window !== 'undefined' ? window.innerWidth : 1200);


  useEffect(() => {
    const handleResize = () => {
      setWindowWidth(window.innerWidth);
    };

    window.addEventListener('resize', handleResize);
    return () => window.removeEventListener('resize', handleResize);
  }, []);

  // Determine breakpoint
  const isMobile = windowWidth < 768;
  const isTablet = windowWidth >= 768 && windowWidth < 1024;
  const isDesktop = windowWidth >= 1024;

  // Dynamic styles based on breakpoint
  const containerStyles: React.CSSProperties = {
    width: '100%',
    maxWidth: isMobile ? '100%' : `${maxWidth}px`,
    margin: centerContent ? '0 auto' : '0',
    padding: isMobile ? '10px' : padding,
    boxSizing: 'border-box'
  };

  const gridStyles: React.CSSProperties = {
    display: 'grid',
    gridTemplateColumns: isMobile
      ? '1fr'
      : isTablet
        ? '1fr 1fr'
        : 'repeat(auto-fit, minmax(300px, 1fr))',
    gap: isMobile ? '15px' : '20px',
    width: '100%'
  };

  return (
    <div
      className={`responsive-container ${className}`}
      style={containerStyles}
      data-breakpoint={isMobile ? 'mobile' : isTablet ? 'tablet' : 'desktop'}
    >
      <div style={gridStyles} role="main" id="main-content">
        {children}
      </div>

      {/* Responsive utilities */}
      <div
        className="responsive-indicator"
        style={{
          position: 'fixed',
          bottom: '10px',
          right: '10px',
          background: '#007cba',
          color: 'white',
          padding: '5px 10px',
          borderRadius: '4px',
          fontSize: '12px',
          zIndex: 1000,
          display: 'none' // Hidden by default, can be shown during development
        }}
      >
        {isMobile ? 'Mobile' : isTablet ? 'Tablet' : 'Desktop'}
      </div>
    </div>
  );
};

// Additional responsive layout components
export const ResponsiveCard: React.FC<PropsWithChildren<{
  title?: string;
  className?: string;
  padding?: string
}>> = ({ title, className = '', padding = '20px', children }) => {
  return (
    <div
      className={`responsive-card ${className}`}
      style={{
        background: 'white',
        border: '1px solid #ddd',
        borderRadius: '8px',
        padding,
        boxShadow: '0 2px 4px rgba(0,0,0,0.1)',
        minHeight: '100px'
      }}
    >
      {title && (
        <h3 style={{
          margin: 0,
          marginBottom: '15px',
          fontSize: '18px',
          fontWeight: 'bold',
          color: '#333'
        }}>
          {title}
        </h3>
      )}
      {children}
    </div>
  );
};

export const ResponsiveGrid: React.FC<PropsWithChildren<{
  columns?: number;
  gap?: string;
  className?: string;
}>> = ({ columns = 3, gap = '20px', className = '', children }) => {
  const [windowWidth, setWindowWidth] = useState(typeof window !== 'undefined' ? window.innerWidth : 1200);

  useEffect(() => {
    const handleResize = () => {
      setWindowWidth(window.innerWidth);
    };

    window.addEventListener('resize', handleResize);
    return () => window.removeEventListener('resize', handleResize);
  }, []);

  const cols = windowWidth < 768 ? 1 : windowWidth < 1024 ? 2 : columns;

  return (
    <div
      className={`responsive-grid ${className}`}
      style={{
        display: 'grid',
        gridTemplateColumns: `repeat(${cols}, 1fr)`,
        gap,
        width: '100%'
      }}
    >
      {children}
    </div>
  );
};

export default ResponsiveContainer;