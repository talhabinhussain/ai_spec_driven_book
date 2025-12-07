import React, { PropsWithChildren } from 'react';

interface AccessibilityWrapperProps {
  title: string;
  description: string;
  dataSummary?: string;
  skipLinkText?: string;
}

const AccessibilityWrapper: React.FC<PropsWithChildren<AccessibilityWrapperProps>> = ({
  title,
  description,
  dataSummary = '',
  skipLinkText = 'Skip to content',
  children
}) => {
  return (
    <div className="accessibility-wrapper" style={{ position: 'relative' }}>
      {/* Skip link for keyboard navigation */}
      <a
        href="#main-content"
        className="skip-link"
        style={{
          position: 'absolute',
          top: '-40px',
          left: '0',
          background: '#000',
          color: '#fff',
          padding: '8px',
          textDecoration: 'none',
          zIndex: 1000
        }}
      >
        {skipLinkText}
      </a>

      {/* ARIA-compliant visualization container */}
      <div
        role="region"
        aria-labelledby="visualization-title"
        aria-describedby="visualization-description"
        tabIndex={0}
        style={{ outline: 'none' }}
      >
        <h3 id="visualization-title" style={{
          fontSize: '16px',
          fontWeight: 'bold',
          margin: 0,
          marginBottom: '8px'
        }}>
          {title}
        </h3>

        <p id="visualization-description" style={{
          fontSize: '14px',
          color: '#666',
          margin: 0,
          marginBottom: '12px'
        }}>
          {description}
        </p>

        {/* Data table alternative for screen readers */}
        {dataSummary && (
          <div
            className="sr-only"
            style={{
              position: 'absolute',
              left: '-10000px',
              top: 'auto',
              width: '1px',
              height: '1px',
              overflow: 'hidden'
            }}
            aria-hidden="true"
          >
            <h4>Data Summary</h4>
            <p>{dataSummary}</p>
          </div>
        )}

        {/* Visualization content */}
        <div id="visualization-content">
          {children}
        </div>
      </div>

      {/* Focus trap for modal-like components */}
      <div
        aria-hidden="true"
        style={{
          position: 'absolute',
          width: '1px',
          height: '1px',
          padding: '0',
          margin: '-1px',
          overflow: 'hidden',
          clip: 'rect(0, 0, 0, 0)',
          whiteSpace: 'nowrap',
          border: '0'
        }}
      >
        Focus trap element
      </div>
    </div>
  );
};

// Additional accessibility utilities
export const HighContrastMode = ({ children }: PropsWithChildren<{}>) => {
  return (
    <div className="high-contrast-mode" style={{
      filter: 'contrast(1.5) brightness(1.2)'
    }}>
      {children}
    </div>
  );
};

export const TextAlternative = ({ text }: { text: string }) => {
  return (
    <div
      className="text-alternative"
      style={{
        padding: '16px',
        border: '1px solid #ccc',
        borderRadius: '4px',
        backgroundColor: '#f9f9f9',
        fontSize: '14px',
        lineHeight: '1.5'
      }}
      aria-hidden="true"
    >
      <h4>Text Alternative</h4>
      <p>{text}</p>
    </div>
  );
};

export default AccessibilityWrapper;