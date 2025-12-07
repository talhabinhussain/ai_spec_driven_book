import React, { useEffect, useRef } from 'react';
import * as d3 from 'd3';

const RobotMetricsChart = ({
  title = "Robot Performance Metrics",
  description = "Visualization of robot performance metrics over time",
  data = [
    { metric: 'Accuracy', value: 87, max: 100 },
    { metric: 'Speed', value: 75, max: 100 },
    { metric: 'Stability', value: 92, max: 100 },
    { metric: 'Efficiency', value: 80, max: 100 },
    { metric: 'Responsiveness', value: 85, max: 100 }
  ]
}) => {
  const svgRef = useRef(null);
  const containerRef = useRef(null);

  useEffect(() => {
    if (!containerRef.current || !data.length) return;

    // Clear previous chart
    d3.select(svgRef.current).selectAll("*").remove();

    const margin = { top: 20, right: 30, bottom: 40, left: 40 };
    const width = containerRef.current.clientWidth - margin.left - margin.right;
    const height = 300 - margin.top - margin.bottom;

    // Create SVG
    const svg = d3.select(svgRef.current)
      .attr("width", width + margin.left + margin.right)
      .attr("height", height + margin.top + margin.bottom)
      .append("g")
      .attr("transform", `translate(${margin.left},${margin.top})`);

    // Scales
    const x = d3.scaleBand()
      .domain(data.map(d => d.metric))
      .range([0, width])
      .padding(0.1);

    const y = d3.scaleLinear()
      .domain([0, d3.max(data, d => d.max)])
      .nice()
      .range([height, 0]);

    // Create bars
    svg.selectAll(".bar")
      .data(data)
      .enter().append("rect")
      .attr("class", "bar")
      .attr("x", d => x(d.metric))
      .attr("width", x.bandwidth())
      .attr("y", d => y(d.value))
      .attr("height", d => height - y(d.value))
      .attr("fill", d => {
        // Color based on value
        if (d.value >= 90) return "#4CAF50"; // Green
        if (d.value >= 75) return "#FFC107"; // Amber
        return "#F44336"; // Red
      })
      .attr("rx", 4)
      .attr("ry", 4);

    // Add value labels on bars
    svg.selectAll(".label")
      .data(data)
      .enter().append("text")
      .attr("class", "label")
      .attr("x", d => x(d.metric) + x.bandwidth() / 2)
      .attr("y", d => y(d.value) - 5)
      .attr("text-anchor", "middle")
      .style("font-size", "12px")
      .style("fill", "#333")
      .style("font-weight", "bold")
      .text(d => `${d.value}%`);

    // Add X axis
    svg.append("g")
      .attr("transform", `translate(0,${height})`)
      .call(d3.axisBottom(x));

    // Add Y axis
    svg.append("g")
      .call(d3.axisLeft(y).ticks(5));

    // Add Y axis label
    svg.append("text")
      .attr("transform", "rotate(-90)")
      .attr("y", 0 - margin.left)
      .attr("x", 0 - (height / 2))
      .attr("dy", "1em")
      .style("text-anchor", "middle")
      .style("font-size", "12px")
      .text("Percentage");

    // Add chart title
    svg.append("text")
      .attr("x", width / 2)
      .attr("y", 0 - (margin.top / 2))
      .attr("text-anchor", "middle")
      .style("font-size", "14px")
      .style("font-weight", "bold")
      .text(title);

    // Handle resize
    const handleResize = () => {
      // For simplicity, we'll recreate the chart on resize
      if (containerRef.current) {
        d3.select(svgRef.current).selectAll("*").remove();
        // The effect will re-run automatically when container size changes
      }
    };

    window.addEventListener('resize', handleResize);

    return () => {
      window.removeEventListener('resize', handleResize);
    };
  }, [data]);

  return (
    <div className="metrics-chart-container" style={{ width: '100%', position: 'relative' }}>
      <div ref={containerRef} style={{ width: '100%', overflowX: 'auto' }}>
        <svg ref={svgRef} style={{ width: '100%', height: '300px' }} />
      </div>
      <div style={{ marginTop: '10px', textAlign: 'center', fontSize: '14px', color: '#666' }}>
        {description}
      </div>
    </div>
  );
};

export default RobotMetricsChart;