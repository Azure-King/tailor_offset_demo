#pragma once

// 只包含 tailor.h，让它管理所有依赖
#include <tailor.h>
#include <optional>
#include <array>
#include <vector>
#include <functional>

namespace tailor_offset {
	// 前置声明
	template <typename Curve, typename T>
	class CurveOffseter;

	// 使用 tailor 库中的常量（避免重复定义）

	/**
	 * @brief 偏置连接段结果
	 */
	template <typename CurveType>
	class OffsetJoinResult {
	public:
		void Push(const CurveType& curve) {
			data_.push_back(curve);
		}

		auto begin() const { return data_.begin(); }
		auto end() const { return data_.end(); }

		size_t Size() const { return data_.size(); }
		bool IsConvexJoin() const { return isConvex_; }
		void SetConvexJoin(bool v) { isConvex_ = v; }

	private:
		std::vector<CurveType> data_;
		bool isConvex_ = false;
	};

	/**
	 * @brief 闭合曲线偏置结果
	 */
	template <typename CurveType>
	class OffsetClosedResult {
	public:
		using Container = std::vector<CurveType>;

		void Push(const CurveType& curve) {
			data_.push_back(curve);
		}

		auto begin() const { return data_.begin(); }
		auto end() const { return data_.end(); }

		size_t Size() const { return data_.size(); }

	private:
		std::vector<CurveType> data_;
	};

	/**
	 * @brief ArcSegment 曲线偏置器特化
	 */
	template <typename PType, typename T, typename UserData>
	class CurveOffseter<tailor::ArcSegment<PType, T, UserData>, T> {
	public:
		using CurveType = tailor::ArcSegment<PType, T, UserData>;
		using PointType = typename CurveType::PointType;
		using ArcTraits = tailor::ArcSegmentTraits<CurveType>;
		using JoinResult = OffsetJoinResult<CurveType>;
		using ClosedResult = OffsetClosedResult<CurveType>;

		/// 输出边的来源类型
		enum class EdgeTag {
			OffsetEdge,       // 直接偏置的边，对应输入曲线的某条边
			JoinConvex,       // 凸点连接弧段
			JoinConcaveLine1, // 凹点连接：从偏置终点到原始顶点的直线段
			JoinConcaveLine2  // 凹点连接：从原始顶点到偏置起点的直线段
		};

		/// 凸角偏置方式
		enum class JoinConvexStyle {
			Arc,         // 圆弧（默认）：生成凸点连接弧
			Extend,      // 延长 - 切向：沿端点切线方向延长直到相交
			ExtendShape  // 延长 - 形状：沿曲线自身几何（弧延圆、线延线）直到相交
		};

		/// 偏置边回调：sourceIndex=对应输入曲线索引，curve=生成的偏置曲线（可修改属性）
		using OffsetEdgeCallback = std::function<void(int sourceIndex, CurveType& curve)>;
		/// 凸点连接弧回调：sourceIndex=前一段曲线索引，joinVertex=凸点所在的原始顶点，curve=生成的凸点连接弧（可修改属性）
		using JoinConvexCallback = std::function<void(int sourceIndex, const PointType& joinVertex, CurveType& curve)>;
		/// 凹点连接线回调：sourceIndex=前一段曲线索引，lineIdx=0或1，curve=生成的连接线（可修改属性）
		using JoinConcaveCallback = std::function<void(int sourceIndex, int lineIdx, CurveType& curve)>;

		/// 分离的静态回调，由外部设置
		static OffsetEdgeCallback s_onOffsetEdge;
		static JoinConvexCallback s_onJoinConvex;
		static JoinConcaveCallback s_onJoinConcave;
		/// 凸角偏置方式选择
		static JoinConvexStyle s_joinConvexStyle;

		static ClosedResult OffsetClosed(
			const std::vector<CurveType>& curves,
			T distance,
			bool ccw) {
			ClosedResult result;
			PointUtils pUtils;

			size_t n = curves.size();
			if (n < 2) {
				return result;
			}

			std::vector<std::optional<CurveType>> offsetCurves(n);
			for (size_t i = 0; i < n; ++i) {
				offsetCurves[i] = OffsetCurve(curves[i], distance, ccw);
			}

			for (size_t i = 0; i < n; ++i) {
				size_t next = (i + 1) % n;

				if (offsetCurves[i].has_value()) {
					auto curve = offsetCurves[i].value();
					if (s_onOffsetEdge) {
						s_onOffsetEdge(static_cast<int>(i), curve);
					}
					result.Push(curve);
				}

				auto joinResult = OffsetJoin(
					curves[i], curves[next],
					offsetCurves[i],
					offsetCurves[next],
					distance,
					ccw);

				bool joinIsConvex = joinResult.IsConvexJoin();
				int joinIdx = 0;
				for (auto joinCurve : joinResult) {
				if (joinIsConvex) {
					if (s_onJoinConvex) {
						s_onJoinConvex(static_cast<int>(i), curves[i].Point1(), joinCurve);
					}
					} else {
						if (s_onJoinConcave) {
							s_onJoinConcave(static_cast<int>(i), joinIdx, joinCurve);
						}
					}
					result.Push(joinCurve);
					++joinIdx;
				}
			}

			return result;
		}

	private:
		using PointUtils = tailor::PointUtils<PointType>;

		static CurveType ConstructOffsetCurve(
			const PointType& p0, const PointType& p1, T bulge,
			const CurveType& from) {
			if constexpr (std::is_same_v<UserData, void>) {
				return CurveType(p0, p1, bulge);
			} else {
				return CurveType(p0, p1, bulge, from.Data());
			}
		}

		static std::optional<CurveType> OffsetCurve(const CurveType& curve, T distance, bool ccw) {
			if (ArcTraits::IsArc(curve)) {
				return OffsetArc(curve, distance, ccw);
			} else {
				return OffsetLine(curve, distance, ccw);
			}
		}

		static std::optional<CurveType> OffsetLine(const CurveType& line, T distance, bool ccw) {
			PointUtils pUtils;
			auto p0 = line.Point0();
			auto p1 = line.Point1();

			auto dir = pUtils.Sub(p1, p0);
			auto len = pUtils.Len(dir);

			if (len < 1e-10) {
				return std::nullopt;
			}

			auto norm = pUtils.Normlize(dir);
			// 沿着曲线方向向左偏置
			auto nx = pUtils.Y(norm) * distance;
			auto ny = -pUtils.X(norm) * distance;

			auto newP0 = pUtils.Add(p0, pUtils.ConstructPoint(nx, ny));
			auto newP1 = pUtils.Add(p1, pUtils.ConstructPoint(nx, ny));

			return ConstructOffsetCurve(newP0, newP1, 0, line);
		}

		static std::optional<CurveType> OffsetArc(const CurveType& arc, T distance, bool ccw) {
			PointUtils pUtils;
			ArcTraits traits;
			auto center = traits.Center(arc);
			auto radius = traits.Radius(arc);
			bool arcCCW = ArcTraits::CCW(arc);

			// 沿着曲线方向向左偏置(偏置距离为负，则为向右)
			T newRadius;
			if (arcCCW) {
				newRadius = radius + distance;
			} else {
				newRadius = radius - distance;
			}

			if (newRadius <= 0) {
				return std::nullopt;
			}

			auto p0 = arc.Point0();
			auto p1 = arc.Point1();

			auto dir0 = pUtils.Sub(p0, center);
			auto len0 = pUtils.Len(dir0);
			auto normDir0 = pUtils.Divide(dir0, len0);

			auto newP0 = pUtils.Add(center, pUtils.Mult(normDir0, newRadius));

			auto dir1 = pUtils.Sub(p1, center);
			auto len1 = pUtils.Len(dir1);
			auto normDir1 = pUtils.Divide(dir1, len1);

			auto newP1 = pUtils.Add(center, pUtils.Mult(normDir1, newRadius));

			return ConstructOffsetCurve(newP0, newP1, arc.Bulge(), arc);
		}

		static PointType GetTangent(const CurveType& curve, const PointType& p) {
			PointUtils pUtils;
			if (!ArcTraits::IsArc(curve)) {
				return pUtils.Normlize(pUtils.Sub(curve.Point1(), curve.Point0()));
			} else {
				ArcTraits traits;
				auto center = traits.Center(curve);
				auto op = pUtils.Sub(p, center);
				bool ccw = ArcTraits::CCW(curve);
				auto tangent = pUtils.ConstructPoint(-pUtils.Y(op), pUtils.X(op));
				if (!ccw) {
					tangent = pUtils.Mult(tangent, T(-1));
				}
				return pUtils.Normlize(tangent);
			}
		}

		static PointType GetNormal(const PointType& tangent, const CurveType& curve, T distance) {
			PointUtils pUtils;
			PointType normal = pUtils.ConstructPoint(-pUtils.Y(tangent), pUtils.X(tangent));
			if (distance < 0) {
				normal = pUtils.Mult(normal, T(-1));
			}
			return normal;
		}

		static bool IsConvexPoint(const PointType& normalAB, const PointType& normalBC, T distance, bool ccw) {
			PointUtils pUtils;
			auto cross = pUtils.Cross(normalAB, normalBC);
			// 凸点判断：
			// - 向外偏置时（distance > 0）需要凸点连接
			// - 向内偏置时（distance < 0）需要凹点连接（直接连接回原顶点）
			// CCW 曲线：cross > 0 时法向外扩（向外偏置时为凸点）
			// CW 曲线：cross < 0 时法向外扩（向外偏置时为凸点）
			// 向外偏置凸点条件：(cross > 0 && ccw) || (cross < 0 && !ccw)
			bool isOuterConvex = (cross > 0 && ccw) || (cross < 0 && !ccw);

			// 回退曲线特殊处理：cross ≈ 0 时，法向可能相反
			constexpr T eps = T(1e-10);
			if (std::abs(cross) < eps) {
				// 检查法向是否相反（回退曲线的特征）
				auto dot = pUtils.X(normalAB) * pUtils.X(normalBC) + pUtils.Y(normalAB) * pUtils.Y(normalBC);
				if (dot < T(-0.9)) {  // 法向几乎相反
					// 回退曲线：向外偏置为凸点，向内偏置为凹点
					return distance > 0;
				}
				// 否则法向相同或接近，不应该发生，保守返回 false
				return false;
			}

			// 向外偏置时 isOuterConvex 为凸点，向内偏置时则相反
			return (distance > 0) ? isOuterConvex : !isOuterConvex;
		}

		static std::optional<CurveType> ConstructJoinArc(
			const PointType& p0, const PointType& p1,
			const PointType& center,
			const CurveType& fromAB, const CurveType& fromBC,
			T distance,
			bool ccw) {
			PointUtils pUtils;

			if (pUtils.IsSamePosition(p0, p1, 1e-10)) {
				return std::nullopt;
			}

			auto v0 = pUtils.Sub(p0, center);
			auto v1 = pUtils.Sub(p1, center);

			using std::atan2;
			auto angle0 = atan2(pUtils.Y(v0), pUtils.X(v0));
			auto angle1 = atan2(pUtils.Y(v1), pUtils.X(v1));

			double dAngle = angle1 - angle0;
			// 凸弧应覆盖外角（可能 > π），根据多边形方向确定正确的弧向
			// CCW 多边形 → 正 bulge (CCW)，CW 多边形 → 负 bulge (CW)
			if (ccw) {
				// CCW: 确保 dAngle ∈ (0, 2π)
				if (dAngle <= T(0)) dAngle += tailor::TAILOR_2PI;
			} else {
				// CW: 确保 dAngle ∈ (-2π, 0)
				if (dAngle >= T(0)) dAngle -= tailor::TAILOR_2PI;
			}

			using std::tan;
			T bulge = static_cast<T>(tan(dAngle / 4.0));

			return ConstructOffsetCurve(p0, p1, bulge, fromAB);
		}

		// ============================================================
		// 延长求交辅助方法（JoinConvexStyle::Extend）
		// ============================================================

		/// 获取偏置曲线在端点的延长切线方向
		/// forward=true: 从终点向前（offsetAB 的 Point1）
		/// forward=false: 从起点向后（offsetBC 的 Point0 的反方向）
		static PointType GetOffsetExtendTangent(const CurveType& curve, bool forward) {
			PointUtils pUtils;
			if (!ArcTraits::IsArc(curve)) {
				auto dir = pUtils.Normlize(pUtils.Sub(curve.Point1(), curve.Point0()));
				return forward ? dir : pUtils.Mult(dir, T(-1));
			} else {
				ArcTraits traits;
				auto center = traits.Center(curve);
				auto pt = forward ? curve.Point1() : curve.Point0();
				auto op = pUtils.Sub(pt, center);
				bool ccw = ArcTraits::CCW(curve);
				auto tangent = pUtils.ConstructPoint(-pUtils.Y(op), pUtils.X(op));
				if (!ccw) tangent = pUtils.Mult(tangent, T(-1));
				tangent = pUtils.Normlize(tangent);
				return forward ? tangent : pUtils.Mult(tangent, T(-1));
			}
		}

		/// 检查点 X 是否可由圆弧 at 点沿指定方向延长到达
		/// forward=true: 沿弧方向延长; forward=false: 逆弧方向延长
		static bool IsOnArcExtension(const CurveType& arc, const PointType& at,
			const PointType& X, bool forward) {
			ArcTraits traits;
			auto center = traits.Center(arc);
			auto radius = traits.Radius(arc);
			PointUtils pUtils;

			auto vX = pUtils.Sub(X, center);
			if (std::abs(pUtils.Len(vX) - radius) > T(1e-6)) return false;

			using std::atan2;
			auto vAt = pUtils.Sub(at, center);
			T angAt = atan2(pUtils.Y(vAt), pUtils.X(vAt));
			T angX = atan2(pUtils.Y(vX), pUtils.X(vX));
			T dAngle = angX - angAt;
			if (dAngle > tailor::TAILOR_PI) dAngle -= tailor::TAILOR_2PI;
			else if (dAngle < -tailor::TAILOR_PI) dAngle += tailor::TAILOR_2PI;

			bool ccw = ArcTraits::CCW(arc);
			if (forward) {
				return ccw ? (dAngle > -T(1e-10)) : (dAngle < T(1e-10));
			} else {
				return ccw ? (dAngle < T(1e-10)) : (dAngle > -T(1e-10));
			}
		}

		/// 射线与圆求交：p0 + t * dir (t >= 0) 与圆的交点参数 t
		static std::vector<T> RayCircleIntersect(
			const PointType& p0, const PointType& dir,
			const PointType& center, T radius) {
			PointUtils pUtils;
			auto v = pUtils.Sub(p0, center);
			T vx = pUtils.X(v), vy = pUtils.Y(v);
			T dx = pUtils.X(dir), dy = pUtils.Y(dir);
			T b = T(2) * (vx * dx + vy * dy);
			T c = vx * vx + vy * vy - radius * radius;
			T disc = b * b - T(4) * c;

			std::vector<T> result;
			if (disc < T(-1e-10)) return result;
			if (disc < T(0)) disc = T(0);
			T sqrtDisc = std::sqrt(disc);
			T t1 = (-b - sqrtDisc) / T(2);
			T t2 = (-b + sqrtDisc) / T(2);
			if (t1 >= -T(1e-10)) result.push_back(t1);
			if (t2 >= -T(1e-10) && std::abs(t2 - t1) > T(1e-10)) result.push_back(t2);
			return result;
		}

		/// 两圆求交
		static std::vector<PointType> CircleCircleIntersect(
			const PointType& c1, T r1, const PointType& c2, T r2) {
			PointUtils pUtils;
			T dx = pUtils.X(c2) - pUtils.X(c1);
			T dy = pUtils.Y(c2) - pUtils.Y(c1);
			T d = std::sqrt(dx * dx + dy * dy);

			std::vector<PointType> result;
			if (d > r1 + r2 + T(1e-10)) return result;
			if (d < std::abs(r1 - r2) - T(1e-10)) return result;
			if (d < T(1e-10)) return result;

			T a = (r1 * r1 - r2 * r2 + d * d) / (T(2) * d);
			T h2 = r1 * r1 - a * a;
			if (h2 < T(-1e-10)) return result;
			if (h2 < T(0)) h2 = T(0);
			T h = std::sqrt(h2);

			T midX = pUtils.X(c1) + a * dx / d;
			T midY = pUtils.Y(c1) + a * dy / d;
			T rx = -dy / d * h;
			T ry = dx / d * h;

			result.push_back(pUtils.ConstructPoint(midX + rx, midY + ry));
			if (h > T(1e-10))
				result.push_back(pUtils.ConstructPoint(midX - rx, midY - ry));
			return result;
		}

		/// 在指定圆弧的圆上，从 p0 到 pX 沿弧方向构造一段弧
		/// center/radius/ccw 来自参考弧 arc，确保新弧在老弧所在的圆上
		/// forward=true: p0 起点，pX 终点，沿弧方向延伸
		/// forward=false: p0 终点，pX 起点，逆弧方向延伸
		static CurveType ConstructArcOnCircle(
			const PointType& p0, const PointType& pX,
			const CurveType& arc, bool forward)
		{
			ArcTraits traits;
			auto center = traits.Center(arc);
			T radius = traits.Radius(arc);
			bool arcCCW = ArcTraits::CCW(arc);
			PointUtils pUtils;

			using std::atan2;
			auto v0 = pUtils.Sub(p0, center);
			auto vX = pUtils.Sub(pX, center);
			T ang0 = atan2(pUtils.Y(v0), pUtils.X(v0));
			T angX = atan2(pUtils.Y(vX), pUtils.X(vX));

			// dAngle: 沿弧方向的角跨度
			// forward=true:  从 p0 到 pX 沿弧方向
			// forward=false: 从 pX 到 p0 沿弧方向（即逆弧从 p0 到 pX）
			T dAngle;
			if (forward) {
				dAngle = angX - ang0;        // CCW from p0 to pX
			} else {
				dAngle = ang0 - angX;        // CCW from pX to p0 (forward for CCW arc)
			}

			// 规范化到 (-π, π]，取最小角跨度
			if (dAngle > tailor::TAILOR_PI) dAngle -= tailor::TAILOR_2PI;
			else if (dAngle < -tailor::TAILOR_PI) dAngle += tailor::TAILOR_2PI;

			// 确保方向与参考弧一致
			if (arcCCW) {
				if (dAngle < T(1e-10)) dAngle += tailor::TAILOR_2PI;
			} else {
				if (dAngle > -T(1e-10)) dAngle -= tailor::TAILOR_2PI;
			}

			using std::tan;
			T bulge = static_cast<T>(tan(dAngle / 4.0));

			if (forward) {
				return ConstructOffsetCurve(p0, pX, bulge, arc);
			} else {
				return ConstructOffsetCurve(pX, p0, bulge, arc);
			}
		}

		/// 凸角延长-切向求交：沿端点切向延长直到相交
		/// 返回值：交点坐标，若无法求交则 std::nullopt（回退到圆弧）
		/// 
		/// "延长-切向"的含义：在偏置曲线的端点处，沿该端点的切线方向
		/// 向外延伸出一条直线，两条延伸线相交即为连接点。
		/// 所有情况都统一使用射线-射线求交，确保真正沿切向延伸。
		static std::optional<PointType> FindExtendIntersection(
			const PointType& p0, const PointType& p1,
			const CurveType& offsetAB, const CurveType& offsetBC) {
			PointUtils pUtils;

			// 统一获取两条偏置曲线在端点的切线方向
			// dir0: offsetAB 在 p0(终点) 处沿切线方向向前的方向
			// dir1: offsetBC 在 p1(起点) 处沿切线方向向后的方向（反向）
			auto dir0 = GetOffsetExtendTangent(offsetAB, true);
			auto dir1 = GetOffsetExtendTangent(offsetBC, false);

			T tx0 = pUtils.X(dir0), ty0 = pUtils.Y(dir0);
			T tx1 = pUtils.X(dir1), ty1 = pUtils.Y(dir1);
			T p0x = pUtils.X(p0), p0y = pUtils.Y(p0);
			T p1x = pUtils.X(p1), p1y = pUtils.Y(p1);

			T dx = p1x - p0x, dy = p1y - p0y;
			T det = tx0 * ty1 - ty0 * tx1;

			// 两切线方向平行 → 无交点，回退到圆弧
			if (std::abs(det) <= T(1e-10)) return std::nullopt;

			// 解方程: p0 + t*dir0 = p1 + s*dir1
			// → t*dir0 - s*dir1 = p1 - p0
			T t = (dx * ty1 - dy * tx1) / det;
			T s = (ty0 * dx - tx0 * dy) / det;

			// 两个参数都必须 >= -eps（允许微小回退，防止数值误差）
			if (t < -T(1e-10) || s < -T(1e-10)) return std::nullopt;

			return pUtils.ConstructPoint(p0x + t * tx0, p0y + t * ty0);
		}

		// ============================================================
		// 延长-形状求交辅助（JoinConvexStyle::ExtendShape）
		// ============================================================

		/// 凸角延长-形状求交：沿曲线自身几何（弧延圆、线延线）直到相交
		/// 返回：交点坐标，若无法求交则 std::nullopt（回退到圆弧）
		static std::optional<PointType> FindShapeExtendIntersection(
			const PointType& p0, const PointType& p1,
			const CurveType& offsetAB, const CurveType& offsetBC)
		{
			PointUtils pUtils;
			bool abIsArc = ArcTraits::IsArc(offsetAB);
			bool bcIsArc = ArcTraits::IsArc(offsetBC);

			std::vector<PointType> candidates;

			if (!abIsArc && !bcIsArc) {
				// Case 1: 两直线 → 与切向模式相同（直线沿自身 = 沿切向）
				auto dir0 = GetOffsetExtendTangent(offsetAB, true);
				auto dir1 = GetOffsetExtendTangent(offsetBC, false);
				T tx0 = pUtils.X(dir0), ty0 = pUtils.Y(dir0);
				T tx1 = pUtils.X(dir1), ty1 = pUtils.Y(dir1);
				T p0x = pUtils.X(p0), p0y = pUtils.Y(p0);
				T p1x = pUtils.X(p1), p1y = pUtils.Y(p1);
				T dx = p1x - p0x, dy = p1y - p0y;
				T det = tx0 * ty1 - ty0 * tx1;
				if (std::abs(det) > T(1e-10)) {
					T t = (dx * ty1 - dy * tx1) / det;
					T s = (ty0 * dx - tx0 * dy) / det;
					if (t >= -T(1e-10) && s >= -T(1e-10))
						candidates.push_back(
							pUtils.ConstructPoint(p0x + t * tx0, p0y + t * ty0));
				}
			} else if (!abIsArc && bcIsArc) {
				// Case 2: offsetAB 直线 + offsetBC 圆弧 → 直线与圆弧的圆求交
				auto dir0 = GetOffsetExtendTangent(offsetAB, true);
				ArcTraits traits;
				auto bcCenter = traits.Center(offsetBC);
				T bcRadius = traits.Radius(offsetBC);
				auto tVals = RayCircleIntersect(p0, dir0, bcCenter, bcRadius);
				for (T t : tVals) {
					PointType X = pUtils.ConstructPoint(
						pUtils.X(p0) + t * pUtils.X(dir0),
						pUtils.Y(p0) + t * pUtils.Y(dir0));
					if (IsOnArcExtension(offsetBC, p1, X, false))
						candidates.push_back(X);
				}
			} else if (abIsArc && !bcIsArc) {
				// Case 3: offsetAB 圆弧 + offsetBC 直线 → 直线与圆弧的圆求交
				auto dir1 = GetOffsetExtendTangent(offsetBC, false);
				ArcTraits traits;
				auto abCenter = traits.Center(offsetAB);
				T abRadius = traits.Radius(offsetAB);
				auto sVals = RayCircleIntersect(p1, dir1, abCenter, abRadius);
				for (T s : sVals) {
					PointType X = pUtils.ConstructPoint(
						pUtils.X(p1) + s * pUtils.X(dir1),
						pUtils.Y(p1) + s * pUtils.Y(dir1));
					if (IsOnArcExtension(offsetAB, p0, X, true))
						candidates.push_back(X);
				}
			} else {
				// Case 4: 两圆弧 → 两圆求交（与切向不同！切向是切线-切线求交）
				ArcTraits traits;
				auto abCenter = traits.Center(offsetAB);
				T abRadius = traits.Radius(offsetAB);
				auto bcCenter = traits.Center(offsetBC);
				T bcRadius = traits.Radius(offsetBC);
				auto pts = CircleCircleIntersect(abCenter, abRadius, bcCenter, bcRadius);
				for (const auto& X : pts) {
					if (IsOnArcExtension(offsetAB, p0, X, true) &&
						IsOnArcExtension(offsetBC, p1, X, false))
						candidates.push_back(X);
				}
			}

			if (candidates.empty()) return std::nullopt;
			T bestDist = std::numeric_limits<T>::max();
			PointType best = candidates[0];
			for (const auto& X : candidates) {
				auto v0 = pUtils.Sub(X, p0);
				auto v1 = pUtils.Sub(X, p1);
				T dist = pUtils.Len(v0) + pUtils.Len(v1);
				if (dist < bestDist) { bestDist = dist; best = X; }
			}
			return best;
		}

		// ============================================================
		// 偏置连接入口
		// ============================================================

		static JoinResult OffsetJoin(
			const CurveType& ab, const CurveType& bc,
			const std::optional<CurveType>& offsetAB,
			const std::optional<CurveType>& offsetBC,
			T distance,
			bool ccw) {
			JoinResult result;
			PointUtils pUtils;
			ArcTraits traits;

			PointType B = ab.Point1();

			// 获取圆心：如果是圆弧则取圆心，否则为曲线的端点
			PointType centerAB = ArcTraits::IsArc(ab) ? traits.Center(ab) : B;
			PointType centerBC = ArcTraits::IsArc(bc) ? traits.Center(bc) : B;

			PointType p0;
			if (offsetAB.has_value()) {
				p0 = offsetAB->Point1();
			} else {
				p0 = centerAB;
			}

			PointType p1;
			if (offsetBC.has_value()) {
				p1 = offsetBC->Point0();
			} else {
				p1 = centerBC;
			}

			auto tangentAB = GetTangent(ab, B);
			auto tangentBC = GetTangent(bc, B);
			auto normalAB = GetNormal(tangentAB, ab, distance);
			auto normalBC = GetNormal(tangentBC, bc, distance);

			bool isConvex = IsConvexPoint(normalAB, normalBC, distance, ccw);

			if (isConvex) {
				// 凸点：根据偏置方式选择圆弧或延长
				if ((s_joinConvexStyle == JoinConvexStyle::Extend ||
				     s_joinConvexStyle == JoinConvexStyle::ExtendShape)
					&& offsetAB.has_value() && offsetBC.has_value()) {

					bool isShapeMode = (s_joinConvexStyle == JoinConvexStyle::ExtendShape);
					auto intersection = isShapeMode
						? FindShapeExtendIntersection(p0, p1, *offsetAB, *offsetBC)
						: FindExtendIntersection(p0, p1, *offsetAB, *offsetBC);

					if (intersection.has_value()) {
						auto X = intersection.value();

						// AB 侧连接：从 p0 → X
						if (!pUtils.IsSamePosition(p0, X, T(1e-10))) {
							if (isShapeMode && ArcTraits::IsArc(*offsetAB)) {
								// 形状模式 + AB 是弧 → 在同一个圆上延伸
								auto arcAB = ConstructArcOnCircle(p0, X, *offsetAB, true);
								result.Push(arcAB);
							} else {
								// 切向模式 或 AB 是直线 → 直线连接
								auto line1 = ConstructOffsetCurve(p0, X, 0, *offsetAB);
								result.Push(line1);
							}
						}

						// BC 侧连接：从 X → p1
						if (!pUtils.IsSamePosition(X, p1, T(1e-10))) {
							if (isShapeMode && ArcTraits::IsArc(*offsetBC)) {
								// 形状模式 + BC 是弧 → 在同一个圆上反向延伸
								auto arcBC = ConstructArcOnCircle(p1, X, *offsetBC, false);
								result.Push(arcBC);
							} else {
								// 切向模式 或 BC 是直线 → 直线连接
								auto line2 = ConstructOffsetCurve(X, p1, 0, *offsetBC);
								result.Push(line2);
							}
						}

						result.SetConvexJoin(true);
						return result;
					}
					// 无交点 → 回退到圆弧
				}

				// 圆弧方式（默认或延长回退）
				auto arc = ConstructJoinArc(p0, p1, B, ab, bc, distance, ccw);
				if (arc.has_value()) {
					result.Push(arc.value());
					result.SetConvexJoin(true);
				}
			} else {
				// 凹点：分别连接到原始凹点 B
				if (!pUtils.IsSamePosition(p0, B, 1e-10)) {
					auto line1 = ConstructOffsetCurve(p0, B, 0, ab);
					result.Push(line1);
				}
				if (!pUtils.IsSamePosition(B, p1, 1e-10)) {
					auto line2 = ConstructOffsetCurve(B, p1, 0, bc);
					result.Push(line2);
				}
			}

			return result;
		}
	};
	
	// 静态回调成员定义（模板偏特化的静态成员）
	template <typename PType, typename T, typename UserData>
	typename CurveOffseter<tailor::ArcSegment<PType, T, UserData>, T>::OffsetEdgeCallback
		CurveOffseter<tailor::ArcSegment<PType, T, UserData>, T>::s_onOffsetEdge;
	template <typename PType, typename T, typename UserData>
	typename CurveOffseter<tailor::ArcSegment<PType, T, UserData>, T>::JoinConvexCallback
		CurveOffseter<tailor::ArcSegment<PType, T, UserData>, T>::s_onJoinConvex;
	template <typename PType, typename T, typename UserData>
	typename CurveOffseter<tailor::ArcSegment<PType, T, UserData>, T>::JoinConcaveCallback
		CurveOffseter<tailor::ArcSegment<PType, T, UserData>, T>::s_onJoinConcave;

	template <typename PType, typename T, typename UserData>
	typename CurveOffseter<tailor::ArcSegment<PType, T, UserData>, T>::JoinConvexStyle
		CurveOffseter<tailor::ArcSegment<PType, T, UserData>, T>::s_joinConvexStyle =
			CurveOffseter<tailor::ArcSegment<PType, T, UserData>, T>::JoinConvexStyle::Arc;

} // namespace tailor_offset
