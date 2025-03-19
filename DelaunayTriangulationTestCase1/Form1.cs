using OxyPlot;
using OxyPlot.Series;
using OxyPlot.WindowsForms;
using DelaunayTriangulation;
using OxyPlot.Annotations;
using OxyPlot.Axes;

namespace DelaunayTriangulationTestCase1
{
	public partial class Form1 : Form
	{
		public static Mesh<double, Vertex2>? Mesh;

		PlotView plot1;

		public Form1()
		{
			#region Setup
			this.plot1 = new OxyPlot.WindowsForms.PlotView();
			this.SuspendLayout();
			// 
			// plot1
			// 
			this.plot1.Dock = System.Windows.Forms.DockStyle.Bottom;
			this.plot1.Location = new System.Drawing.Point(0, 0);
			this.plot1.Name = "plot1";
			this.plot1.PanCursor = System.Windows.Forms.Cursors.Hand;
			this.plot1.Size = new System.Drawing.Size(500, 500);
			this.plot1.TabIndex = 0;
			this.plot1.Text = "plot1";
			this.plot1.ZoomHorizontalCursor = System.Windows.Forms.Cursors.SizeWE;
			this.plot1.ZoomRectangleCursor = System.Windows.Forms.Cursors.SizeNWSE;
			this.plot1.ZoomVerticalCursor = System.Windows.Forms.Cursors.SizeNS;
			this.Controls.Add(this.plot1);

			InitializeComponent();
			var model = new PlotModel { Title = "Test" };
			this.plot1.Model = model;
			#endregion Setup

			// Randomly initialize some vertices
			const int NUM_VERTICES = 24;
			Random rand = new Random();
			List<Vertex2> vertices = new List<Vertex2>();
			for (int k = 0; k < NUM_VERTICES; ++k)
				vertices.Add(new Vertex2
				{
					X = rand.NextDouble(),
					Y = rand.NextDouble()
				});

			// Construct a mesh from them
			Mesh = Mesh<double, Vertex2>.Construct(vertices);

			// Display the edges onto the plot
			model.Axes.Add(new LinearAxis
			{
				Position = AxisPosition.Left,
				Minimum = -0.1,
				Maximum = 1.1
			});
			model.Axes.Add(new LinearAxis
			{
				Position = AxisPosition.Bottom,
				Minimum = -0.1,
				Maximum = 1.1
			});

			// Draw edges
			foreach (Edge<double, Vertex2> edge in Mesh.Edges)
			{
				LineAnnotation edgeLine = new LineAnnotation
				{
					Type = LineAnnotationType.LinearEquation,
					LineStyle = LineStyle.Solid,
					MinimumX = Math.Min(edge.Vertex1.X, edge.Vertex2.X),
					MaximumX = Math.Max(edge.Vertex1.X, edge.Vertex2.X),
					MinimumY = Math.Min(edge.Vertex1.Y, edge.Vertex2.Y),
					MaximumY = Math.Max(edge.Vertex1.Y, edge.Vertex2.Y)
				};
				if (edgeLine.MinimumX == edge.Vertex1.X)
				{
					edgeLine.Slope = (edge.Vertex2.Y - edge.Vertex1.Y) / (edgeLine.MaximumX - edgeLine.MinimumX);
					edgeLine.Intercept = edge.Vertex1.Y - edgeLine.MinimumX * edgeLine.Slope;
				}
				else
				{
					edgeLine.Slope = (edge.Vertex1.Y - edge.Vertex2.Y) / (edgeLine.MaximumX - edgeLine.MinimumX);
					edgeLine.Intercept = edge.Vertex2.Y - edgeLine.MinimumX * edgeLine.Slope;
				}
				model.Annotations.Add(edgeLine);
			}

			// Draw triangles
			foreach (Triangle<double, Vertex2> triangle in Mesh.Triangles)
			{
				PolygonAnnotation triangleLines = new PolygonAnnotation
				{
					LineStyle = LineStyle.Dot,
					Stroke = OxyColor.FromRgb(0xFF, 0x00, 0x00),
					StrokeThickness = 2.0,
					Fill = OxyColor.FromArgb(0x10, 0xFF, 0x00, 0x00)
				};
				triangleLines.Points.AddRange(triangle.Vertices.Select(v => new DataPoint(v.X, v.Y)));
				model.Annotations.Add(triangleLines);
			}
			
			// Draw vertices
			foreach (Vertex2 vertex in Mesh.Vertices)
			{
				model.Annotations.Add(new PointAnnotation
				{
					X = vertex.X,
					Y = vertex.Y,
					Size = 3.0,
					Fill = OxyColor.FromRgb(0x00, 0x00, 0x00)
				});
			}
		}
	}
}
