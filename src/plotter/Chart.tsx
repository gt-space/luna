import { Component, createSignal } from 'solid-js';
import {createEffect} from 'solid-js';
import {Chart,registerables, ChartConfiguration} from 'chart.js';
import 'chartjs-adapter-luxon';
import ChartStreaming from 'chartjs-plugin-streaming';
import Zoom from 'chartjs-plugin-zoom';
import { plotterValues } from './PlotterView';

Chart.register(...registerables, Zoom, ChartStreaming);


const ChartComponent: Component<{id: string, index: number}> = (props) => {

    const [thisChart, setThisChart] = createSignal();

    const data = {
        datasets: [
          {
            label: props.id,
            data: [],
            borderColor: "#36A2EB",
            backgroundColor: "#346A8F",
            pointBackgroundColor: "#346A8F",
            pointBorderColor: "#36A2EB",
            pointHoverBackgroundColor: "#346A8F",
            pointHoverBorderColor: "#36A2EB",
          }
        ]
    };

    function resetChartZoom() {
      (thisChart() as Chart).resetZoom();
    }

    const onRefresh = (chart: Chart) => {
        const now = Date.now();
        chart.data.datasets.forEach(async (dataset) => {
          var yVal = plotterValues()[props.index];
          dataset.data.push({
            x: now,
            y: yVal
          });
        });
    };
    const config: ChartConfiguration = {
        type: 'line',
        data: data,
        options: {
          elements: {
              point:{
                  radius: 0
              }
          },
          plugins: {
            legend: {
              labels: {
              color: 'white'
              }
            },
            zoom: {
              zoom: {
                wheel: {
                  enabled: true,
                },
                pinch: {
                  enabled: true
                },
                mode: 'y',
              },
            }
          },
          scales: {
            x: {
              type: 'realtime',
              realtime: {
                  duration: 30000,
                  refresh: 200,
                  delay: 0,
                  frameRate: 20,
                  onRefresh: onRefresh
              },
              grid: {
                color: '#545454',
                borderColor: 'white'
              },
              ticks: {
                color: 'white'
              }
            },
            y: {
              title: {
                  display: true,
                  text: 'Value',
                  color: 'white'
              },
              grid: {
                color: '#545454',
                borderColor: 'white'
              },
              ticks: {
                color: 'white'
              }
            },
          },
          interaction: {
            intersect: false
          }
        }
    };
    createEffect(async () => {
      console.log('test', document.getElementById(props.id) as HTMLCanvasElement);
      const myChart = new Chart(document.getElementById(props.id) as HTMLCanvasElement, config);
      setThisChart(myChart);
    });
    return (
        <div>
          <button class='chart-reset-button' onClick={resetChartZoom}>Reset Zoom</button>
          <div style="display: block; margin: 0px; width: 400px;"><canvas id={props.id} class='chart-tile'></canvas></div>
        </div>
        
    );
}

export default ChartComponent;