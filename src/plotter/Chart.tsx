import { Component } from 'solid-js';
import {createEffect} from 'solid-js';
import {Chart, ChartConfiguration, ChartTypeRegistry} from 'chart.js/auto';
import 'chartjs-adapter-luxon';
import ChartStreaming from 'chartjs-plugin-streaming';
import { plotterValues } from './PlotterView';

Chart.register(ChartStreaming);

const ChartComponent: Component<{id: string, index: number}> = (props) => {
    const data = {
        datasets: [
          {
            label: props.id,
            data: []
          }
        ]
    };
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
            new Chart(document.getElementById(props.id) as HTMLCanvasElement, config);
          })();
    });
    return (
        <div style="display: block; margin: 0px; width: 400px;"><canvas id={props.id} class='chart-tile'></canvas></div>
    );
}

export default ChartComponent;