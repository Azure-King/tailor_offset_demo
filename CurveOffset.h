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

	private:
		std::vector<CurveType> data_;
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

				bool joinIsConvex = (joinResult.Size() == 1);
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
			if (dAngle > tailor::TAILOR_PI) {
				dAngle -= tailor::TAILOR_2PI;
			} else if (dAngle < -tailor::TAILOR_PI) {
				dAngle += tailor::TAILOR_2PI;
			}
			// dAngle 保持在 (-PI, PI] 范围，保证始终生成劣弧 (|bulge| <= 1)

			using std::tan;
			T bulge = static_cast<T>(tan(dAngle / 4.0));

			return ConstructOffsetCurve(p0, p1, bulge, fromAB);
		}

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
				auto arc = ConstructJoinArc(p0, p1, B, ab, bc, distance, ccw);
				if (arc.has_value()) {
					result.Push(arc.value());
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

} // namespace tailor_offset
