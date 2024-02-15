import { Component, createSignal } from 'solid-js';
import {createEffect} from 'solid-js';
import {Chart, ChartConfiguration, ChartTypeRegistry} from 'chart.js/auto';
import 'chartjs-adapter-luxon';
import ChartStreaming from 'chartjs-plugin-streaming';
import Zoom from 'chartjs-plugin-zoom';
import { plotterValues } from './PlotterView';

Chart.register(Zoom);
Chart.register(ChartStreaming);


const ChartComponent: Component<{id: string, index: number}> = (props) => {

    const [thisChart, setThisChart] = createSignal();

    const data = {
        datasets: [
          {
            label: props.id,
            data: []
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
              limits: {
                y: {minRange: 1}
              }
            }
          },
          scales: {
              x: {
              type: 'realtime',
              realtime: {
                  duration: 10000,
                  refresh: 16,
                  delay: 0,
                  frameRate: 55,
                  onRefresh: onRefresh
              }
              },
              y: {
              title: {
                  display: true,
                  text: 'Value'
              }
              }
          },
          interaction: {
            intersect: false
          }
        }
    };
    createEffect(() => {
        (async function() {          
            setThisChart(new Chart(document.getElementById(props.id) as HTMLCanvasElement, config));
          })();
    });
    return (
        <div>
          <button class='chart-reset-button' onClick={resetChartZoom}>Reset Zoom</button>
          <div style="display: block; margin: 0px; width: 400px;"><canvas id={props.id} class='chart-tile'></canvas></div>
        </div>
        
    );
}

export default ChartComponent;